#include <UTFT.h>
#include <URTouch.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include <math.h>

// ============================================================
// RELÉ DIGITAL DE PROTEÇÃO
// ARDUINO MEGA 2560 + IHM TOUCH SCREEN (TFT ILI9341 + XPT2046)
// MEDIÇÃO RMS + EEPROM + DS3231
// FUNÇÕES: ANSI 50 / 51 / 27 / 59
// ============================================================
//
// MENU (conforme fluxograma):
//   1. SUPERVISÃO E OPERAÇÃO
//   2. CONFIGURAÇÃO BÁSICA     (data / hora / atualizar RTC)
//   3. CONFIGURAÇÃO AVANÇADA   (senha 1234)
//        Calibração | TCs | TPs | ANSI 50 | 51 | 27 | 59
//   4. HISTÓRICO DE EVENTOS    (últimos 5 eventos / 2 dias)
//
// LÓGICA GERAL (executada em qualquer tela, exceto calibração):
//   Leitura dos sensores -> fatores de calibração -> RTC/RTP
//   -> ANSI 50 -> ANSI 51 -> ANSI 27 -> ANSI 59
//   -> falha? registra evento + abre disjuntor + atualiza tela
//
// ============================================================
// TFT ILI9341 + TOUCH XPT2046
// ============================================================
UTFT myGLCD(ILI9341_16, 38, 39, 40, 41);
URTouch myTouch(6, 5, 4, 3, 2);
extern uint8_t SmallFont[];

// ============================================================
// RTC DS3231  (SDA = D20 | SCL = D21)
// ============================================================
RTC_DS3231 rtc;

// ============================================================
// PINOS (QUADRO 02 do descritivo)
// ============================================================
const int pinosZMPT[3] = {A0, A1, A2};   // VA, VB, VC
const int pinosACS[3]  = {A3, A4, A5};   // IA, IB, IC

const int PINO_ESTADO_DISJUNTOR = 8;     // Aberto / Fechado
const int PINO_MOLA             = 9;     // Mola carregada
const int PINO_SF6              = 10;    // Nível muito baixo de SF6
const int PINO_FALHAS           = 11;    // Resumo geral de falhas
const int PINO_FECHAR           = 14;    // Saída: comando de fechamento
const int PINO_TRIP             = 15;    // Saída: comando de abertura (trip)

// ------------------------------------------------------------
// AJUSTE DE POLARIDADE DAS ENTRADAS DIGITAIS
// true  = contato fecha para GND (INPUT_PULLUP, ativo em nível BAIXO)
// false = ativo em nível ALTO
// Disjuntor: ativo = FECHADO | Mola: ativo = CARREGADA
// SF6: ativo = NÍVEL MUITO BAIXO | Falhas: ativo = EXISTE FALHA
// ------------------------------------------------------------
const bool ENTRADAS_ATIVAS_EM_BAIXO = true;

// Duração do pulso das saídas de fechamento e trip
const unsigned long PULSO_COMANDO_MS = 500;

// ============================================================
// ADC / REFERÊNCIAS
// ============================================================
const float VREF_ADC = 5.0;
const float ADC_MAX  = 1023.0;

// Valores de referência da calibração (secundários)
const float TENSAO_REFERENCIA   = 115.0;   // V
const float CORRENTE_REFERENCIA = 5.0;     // A

// Secundários nominais de TP e TC
const float TP_SECUNDARIO = 115.0;
const float TC_SECUNDARIO = 5.0;

// ============================================================
// AMOSTRAGEM RMS
// Janela = 1 ciclo completo da rede (60 Hz = 16667 us).
// Para rede de 50 Hz usar 20000.
// ============================================================
const unsigned long JANELA_AMOSTRAGEM_US = 16667UL;
const int AMOSTRAS_CALIBRACAO = 10;        // média na captura de calibração

// ============================================================
// PROTEÇÃO
// ============================================================
// Tempo de confirmação das funções 27 e 59 (evita atuação por
// variações momentâneas - item 3.4.3 / 3.4.4 do descritivo)
const unsigned long CONFIRMACAO_TENSAO_MS = 200;

// ============================================================
// HISTÓRICO
// ============================================================
const uint8_t MAX_EVENTOS = 5;
const uint32_t JANELA_HISTORICO_S = 172800UL;   // 2 dias

// ============================================================
// SENHA DO MENU AVANÇADO
// ============================================================
const char SENHA_AVANCADA[] = "1234";

// ============================================================
// CORES
// ============================================================
#define PRETO    VGA_BLACK
#define BRANCO   VGA_WHITE
#define AZUL     VGA_BLUE
#define VERDE    VGA_GREEN
#define CINZA    VGA_GRAY
#define VERMELHO VGA_RED
#define AMARELO  VGA_YELLOW

// ============================================================
// TELAS
// ============================================================
enum Tela {
  MENU_PRINCIPAL,
  SUPERVISAO,
  CONFIG_BASICA,
  TECLADO,
  MENU_AVANCADO,
  MENU_CALIBRACAO,
  TELA_CALIBRACAO,
  TELA_TC,
  TELA_TP,
  TELA_ANSI50,
  TELA_ANSI51,
  TELA_ANSI27,
  TELA_ANSI59,
  HISTORICO
};

Tela telaAtual = MENU_PRINCIPAL;
Tela telaRetorno = MENU_PRINCIPAL;

enum ModoTeclado {
  TECLADO_DATA,
  TECLADO_HORA,
  TECLADO_SENHA,
  TECLADO_NUMERO
};

enum Parametro {
  PAR_TC,
  PAR_TP,
  PAR_50,
  PAR_51_PICKUP,
  PAR_51_TMS,
  PAR_27,
  PAR_59
};

enum Funcao {
  FUNC_NENHUMA = 0,
  FUNC_ANSI50,
  FUNC_ANSI51,
  FUNC_ANSI27,
  FUNC_ANSI59
};

const uint8_t CURVA_SI = 0;   // Standard Inverse
const uint8_t CURVA_EI = 1;   // Extremely Inverse

// ============================================================
// ESTRUTURAS
// ============================================================
struct Calibracao {
  float zeroRMS;            // offset (0 V / 0 A)
  float referenciaRMS;      // leitura com 115 V / 5 A
  float valorReferencia;
  float fatorK;             // K = valorReferencia / (referenciaRMS - zeroRMS)
  bool zeroCalibrado;
  bool referenciaCalibrada;
  bool kCalculado;
};

struct Config {
  float tcPrimario;         // A
  float tpPrimario;         // V
  float pickup50;           // A (primário)
  float pickup51;           // A (primário)
  float tms51;
  uint8_t curva51;          // CURVA_SI / CURVA_EI
  float pct27;              // % da tensão nominal
  float pct59;              // % da tensão nominal
};

struct Evento {
  uint32_t instante;        // unixtime do DS3231
  uint8_t funcao;
  uint8_t estadoDisjuntor;  // 1 = fechado
  float v[3];
  float i[3];
};

// ============================================================
// EEPROM
// ============================================================
const uint16_t EEPROM_MAGIC_CALIB  = 0x5A38;
const uint16_t EEPROM_MAGIC_CONFIG = 0x5A39;
const uint16_t EEPROM_MAGIC_HIST   = 0x5A3A;

const int EEPROM_END_CALIB  = 0;
const int EEPROM_END_CONFIG = 256;
const int EEPROM_END_HIST   = 512;

struct DadosCalib {
  uint16_t magic;
  Calibracao tensao[3];
  Calibracao corrente[3];
};

struct DadosConfig {
  uint16_t magic;
  Config cfg;
};

struct DadosHist {
  uint16_t magic;
  uint8_t proximo;
  uint8_t total;
  Evento ev[MAX_EVENTOS];
};

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
Calibracao calibTensao[3];
Calibracao calibCorrente[3];
Config cfg;
DadosHist hist;

// Medições (secundário e primário)
float tensaoSec[3]  = {0, 0, 0};
float correnteSec[3] = {0, 0, 0};
float tensaoRMS[3]  = {0, 0, 0};    // primário (V)
float correnteRMS[3] = {0, 0, 0};   // primário (A)

int canalMedicao = 0;
bool rodadaCompleta = false;

// Estados de campo
bool disjuntorFechado = false;
bool molaCarregada = false;
bool sf6Baixo = false;
bool falhaGeral = false;

// Falha atuada
bool falhaAtiva = false;
uint8_t funcaoAtuada = FUNC_NENHUMA;

// Temporizadores de proteção
bool temporizando51 = false;
unsigned long inicio51 = 0;
bool confirmando27 = false;
unsigned long inicio27 = 0;
bool confirmando59 = false;
unsigned long inicio59 = 0;

// Saídas
bool pulsoTrip = false;
bool pulsoFecha = false;
unsigned long inicioPulsoTrip = 0;
unsigned long inicioPulsoFecha = 0;

// Touch
unsigned long ultimoToque = 0;
const unsigned long DEBOUNCE = 300;

// Temporização de tela
unsigned long ultimoRTC = 0;
unsigned long ultimaTela = 0;
unsigned long ultimoLiveCal = 0;
unsigned long ultimoSerial = 0;

// Data / hora
DateTime agoraRTC;
int diaAtual, mesAtual, anoAtual;
int horaAtual, minutoAtual, segundoAtual;

// Valores em edição (config básica)
int ajDia, ajMes, ajAno;
int ajHora, ajMinuto, ajSegundo;

// Teclado
ModoTeclado modoTeclado = TECLADO_NUMERO;
Parametro parametroTeclado = PAR_TC;
const char *tituloTeclado = "";
const char *unidadeTeclado = "";
String entradaTeclado = "";

const int TEC_X[3] = {10, 85, 160};
const int TEC_Y[4] = {62, 102, 142, 182};
const int TEC_W = 70;
const int TEC_H = 34;

// Calibração
int canalCal = 0;   // 0..2 = VA..VC | 3..5 = IA..IC

// Histórico
int idxHist[MAX_EVENTOS];
int qtdHist = 0;
int eventoSel = 0;

// Supervisão
uint8_t funcaoDesenhada = 255;

// ============================================================
// PROTÓTIPOS
// ============================================================
// Desenho
void desenharBotao(int x1, int y1, int x2, int y2, word cor,
                   const char *texto, word corTexto = BRANCO);
void desenharBotaoVoltar();
void cabecalho(const char *titulo);
void escreverPad(const char *txt, int x, int y, int largura);
void escreverCentro(const char *txt, int y, int largura);
void formatarNumero(char *dst, float v);
void mostrarMensagem(const char *linha1, const char *linha2);
void irParaTela(Tela nova);
void desenharMenuPrincipal();
void atualizarRodape();
void desenharSupervisao();
void atualizarTelaSupervisao();
void desenharConfigBasica();
void atualizarRelogioConfig();
void desenharTeclado();
void atualizarCampoTeclado();
void desenharMenuAvancado();
void desenharMenuCalibracao();
void desenharTelaCalibracao();
void atualizarLiveCalibracao();
void desenharRelacao(bool ehTC);
void desenharAnsi50();
void desenharAnsi51();
void desenharAnsiTensao(bool subtensao);
void desenharHistorico();

// Touch
void verificarTouch();
void processarTouch(int x, int y);
void touchMenuPrincipal(int x, int y);
void touchSupervisao(int x, int y);
void touchConfigBasica(int x, int y);
void touchTeclado(int x, int y);
void touchMenuAvancado(int x, int y);
void touchMenuCalibracao(int x, int y);
void touchTelaCalibracao(int x, int y);
void touchRelacao(int x, int y, bool ehTC);
void touchAnsi50(int x, int y);
void touchAnsi51(int x, int y);
void touchAnsiTensao(int x, int y, bool subtensao);
void touchHistorico(int x, int y);

// Teclado
void abrirTecladoNumero(Parametro p, const char *titulo,
                        const char *unidade, Tela retorno);
void abrirTecladoSimples(ModoTeclado modo, Tela retorno);
void processarTeclaNumerica(char tecla);
void confirmarTeclado();
bool aplicarParametro(Parametro p, float valor);

// Medição / proteção
float lerRMS(int pino);
float lerRMSMedio(int pino);
void medirProximoCanal();
void lerEntradas();
bool lerEntrada(int pino);
float relacaoTC();
float relacaoTP();
float calcularTempo51(float m);
void executarProtecoes();
void dispararProtecao(uint8_t funcao);
void resetarTemporizadores();
bool tensoesCalibradas();
bool correntesCalibradas();
const char *nomeFuncao(uint8_t f);

// Comandos / saídas
void acionarTrip();
void acionarFechamento();
void gerenciarSaidas();
void comandoFechar();
void comandoAbrir();
void comandoReset();

// Calibração
Calibracao &calibAtual();
int pinoCalAtual();
void calibrarOffset();
void calibrarReferencia();
void calcularK();

// EEPROM
void inicializarCalibracaoPadrao();
void inicializarConfigPadrao();
void inicializarHistoricoPadrao();
void salvarCalibracaoEEPROM();
void carregarCalibracaoEEPROM();
void salvarConfigEEPROM();
void carregarConfigEEPROM();
void salvarHistoricoEEPROM();
void carregarHistoricoEEPROM();

// RTC / histórico
void lerRTC();
void gravarRTC();
bool validarData(int d, int m, int a);
bool validarHora(int h, int mi, int s);
int diasNoMes(int m, int a);
void montarListaHistorico();
Evento montarEvento(uint8_t funcao);
void gravarEvento(const Evento &e);

// ============================================================
// SETUP
// ============================================================
void setup() {

  Serial.begin(115200);

  // ----------------------------------------------------------
  // SAÍDAS (sempre em nível baixo na partida)
  // ----------------------------------------------------------
  pinMode(PINO_FECHAR, OUTPUT);
  pinMode(PINO_TRIP, OUTPUT);
  digitalWrite(PINO_FECHAR, LOW);
  digitalWrite(PINO_TRIP, LOW);

  // ----------------------------------------------------------
  // ENTRADAS DIGITAIS
  // ----------------------------------------------------------
  if (ENTRADAS_ATIVAS_EM_BAIXO) {
    pinMode(PINO_ESTADO_DISJUNTOR, INPUT_PULLUP);
    pinMode(PINO_MOLA, INPUT_PULLUP);
    pinMode(PINO_SF6, INPUT_PULLUP);
    pinMode(PINO_FALHAS, INPUT_PULLUP);
  }
  else {
    pinMode(PINO_ESTADO_DISJUNTOR, INPUT);
    pinMode(PINO_MOLA, INPUT);
    pinMode(PINO_SF6, INPUT);
    pinMode(PINO_FALHAS, INPUT);
  }

  // ----------------------------------------------------------
  // TFT
  // ----------------------------------------------------------
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);

  // ----------------------------------------------------------
  // TOUCH
  // ----------------------------------------------------------
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);

  // ----------------------------------------------------------
  // RTC
  // ----------------------------------------------------------
  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("ERRO: DS3231 NAO ENCONTRADO!");
  }
  else {
    Serial.println("DS3231 encontrado.");

    if (rtc.lostPower()) {
      Serial.println("RTC perdeu alimentacao.");
      // Ajusta pela data/hora da compilação somente se o RTC
      // tiver perdido a alimentação.
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC ajustado pela data/hora da compilacao.");
    }
  }

  // ----------------------------------------------------------
  // EEPROM: calibração, ajustes e histórico
  // ----------------------------------------------------------
  inicializarCalibracaoPadrao();
  inicializarConfigPadrao();
  inicializarHistoricoPadrao();

  carregarCalibracaoEEPROM();
  carregarConfigEEPROM();
  carregarHistoricoEEPROM();

  lerRTC();
  lerEntradas();

  irParaTela(MENU_PRINCIPAL);
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  // ==========================================================
  // TOUCH
  // ==========================================================
  verificarTouch();

  // ==========================================================
  // ENTRADAS DIGITAIS
  // ==========================================================
  lerEntradas();

  // ==========================================================
  // MEDIÇÃO + PROTEÇÃO (qualquer tela, exceto calibração)
  // Um canal por passagem: o touch é varrido a cada ~17 ms.
  // ==========================================================
  bool emCalibracao = (telaAtual == MENU_CALIBRACAO ||
                       telaAtual == TELA_CALIBRACAO);

  if (!emCalibracao) {
    medirProximoCanal();

    if (rodadaCompleta) {
      executarProtecoes();
    }
  }

  gerenciarSaidas();

  // ==========================================================
  // RTC (1 s)
  // ==========================================================
  if (millis() - ultimoRTC >= 1000) {
    ultimoRTC = millis();
    lerRTC();

    if (telaAtual == MENU_PRINCIPAL) {
      atualizarRodape();
    }
    else if (telaAtual == CONFIG_BASICA) {
      atualizarRelogioConfig();
    }
  }

  // ==========================================================
  // TELA DE SUPERVISÃO (200 ms)
  // ==========================================================
  if (telaAtual == SUPERVISAO &&
      millis() - ultimaTela >= 200) {
    ultimaTela = millis();
    atualizarTelaSupervisao();
  }

  // ==========================================================
  // LEITURA AO VIVO NA CALIBRAÇÃO (500 ms)
  // ==========================================================
  if (telaAtual == TELA_CALIBRACAO &&
      millis() - ultimoLiveCal >= 500) {
    ultimoLiveCal = millis();
    atualizarLiveCalibracao();
  }

  // ==========================================================
  // SERIAL (1 s)
  // ==========================================================
  if (millis() - ultimoSerial >= 1000) {
    ultimoSerial = millis();

    Serial.print("VA=");  Serial.print(tensaoRMS[0], 1);
    Serial.print(" VB="); Serial.print(tensaoRMS[1], 1);
    Serial.print(" VC="); Serial.print(tensaoRMS[2], 1);
    Serial.print(" | IA="); Serial.print(correnteRMS[0], 2);
    Serial.print(" IB=");   Serial.print(correnteRMS[1], 2);
    Serial.print(" IC=");   Serial.print(correnteRMS[2], 2);
    Serial.print(" | DJ=");  Serial.print(disjuntorFechado ? "F" : "A");
    Serial.print(" FALHA="); Serial.println(nomeFuncao(funcaoAtuada));
  }
}

// ============================================================
// TOUCH
// ============================================================
void verificarTouch() {

  if (!myTouch.dataAvailable()) return;

  myTouch.read();

  int x = myTouch.getX();
  int y = myTouch.getY();

  if (x >= 0 && x <= 320 && y >= 0 && y <= 240) {

    if (millis() - ultimoToque > DEBOUNCE) {
      ultimoToque = millis();
      processarTouch(x, y);
    }
  }
}

bool dentro(int x, int y, int x1, int y1, int x2, int y2) {
  return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
}

bool tocouVoltar(int x, int y) {
  return dentro(x, y, 80, 205, 240, 232);
}

void processarTouch(int x, int y) {

  switch (telaAtual) {
    case MENU_PRINCIPAL:   touchMenuPrincipal(x, y);   break;
    case SUPERVISAO:       touchSupervisao(x, y);      break;
    case CONFIG_BASICA:    touchConfigBasica(x, y);    break;
    case TECLADO:          touchTeclado(x, y);         break;
    case MENU_AVANCADO:    touchMenuAvancado(x, y);    break;
    case MENU_CALIBRACAO:  touchMenuCalibracao(x, y);  break;
    case TELA_CALIBRACAO:  touchTelaCalibracao(x, y);  break;
    case TELA_TC:          touchRelacao(x, y, true);   break;
    case TELA_TP:          touchRelacao(x, y, false);  break;
    case TELA_ANSI50:      touchAnsi50(x, y);          break;
    case TELA_ANSI51:      touchAnsi51(x, y);          break;
    case TELA_ANSI27:      touchAnsiTensao(x, y, true);  break;
    case TELA_ANSI59:      touchAnsiTensao(x, y, false); break;
    case HISTORICO:        touchHistorico(x, y);       break;
  }
}

// ============================================================
// NAVEGAÇÃO
// ============================================================
void irParaTela(Tela nova) {

  bool eraCalibracao = (telaAtual == MENU_CALIBRACAO ||
                        telaAtual == TELA_CALIBRACAO);
  bool seraCalibracao = (nova == MENU_CALIBRACAO ||
                         nova == TELA_CALIBRACAO);

  // Ao sair da calibração, reinicia a varredura de medição
  if (eraCalibracao && !seraCalibracao) {
    canalMedicao = 0;
    rodadaCompleta = false;
    resetarTemporizadores();
  }

  // Ao entrar na calibração, zera temporizadores
  if (!eraCalibracao && seraCalibracao) {
    resetarTemporizadores();
  }

  telaAtual = nova;

  switch (nova) {
    case MENU_PRINCIPAL:  desenharMenuPrincipal();   break;
    case SUPERVISAO:      desenharSupervisao();      break;
    case CONFIG_BASICA:   desenharConfigBasica();    break;
    case TECLADO:         desenharTeclado();         break;
    case MENU_AVANCADO:   desenharMenuAvancado();    break;
    case MENU_CALIBRACAO: desenharMenuCalibracao();  break;
    case TELA_CALIBRACAO: desenharTelaCalibracao();  break;
    case TELA_TC:         desenharRelacao(true);     break;
    case TELA_TP:         desenharRelacao(false);    break;
    case TELA_ANSI50:     desenharAnsi50();          break;
    case TELA_ANSI51:     desenharAnsi51();          break;
    case TELA_ANSI27:     desenharAnsiTensao(true);  break;
    case TELA_ANSI59:     desenharAnsiTensao(false); break;
    case HISTORICO:
      montarListaHistorico();
      eventoSel = 0;
      desenharHistorico();
      break;
  }
}

// ============================================================
// FUNÇÕES AUXILIARES DE DESENHO
// ============================================================
void cabecalho(const char *titulo) {
  myGLCD.clrScr();
  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);
  myGLCD.setFont(SmallFont);
  myGLCD.print(titulo, CENTER, 5);
}

void desenharBotao(int x1, int y1, int x2, int y2, word cor,
                   const char *texto, word corTexto) {

  myGLCD.setColor(cor);
  myGLCD.fillRect(x1, y1, x2, y2);

  int tx = x1 + ((x2 - x1) - (int)strlen(texto) * 8) / 2;
  int ty = y1 + ((y2 - y1) - 12) / 2;
  if (tx < x1) tx = x1;

  myGLCD.setColor(corTexto);
  myGLCD.setBackColor(cor);
  myGLCD.print(texto, tx, ty);

  myGLCD.setBackColor(PRETO);
  myGLCD.setColor(BRANCO);
}

void desenharBotaoVoltar() {
  desenharBotao(80, 205, 240, 232, CINZA, "VOLTAR");
}

// Imprime texto preenchido com espaços (apaga resíduos sem fillRect)
void escreverPad(const char *txt, int x, int y, int largura) {

  char buf[48];
  int i = 0;

  if (largura > 46) largura = 46;

  while (txt[i] != 0 && i < largura) {
    buf[i] = txt[i];
    i++;
  }
  while (i < largura) {
    buf[i++] = ' ';
  }
  buf[i] = 0;

  myGLCD.print(buf, x, y);
}

// Imprime texto centralizado em campo de 'largura' caracteres
void escreverCentro(const char *txt, int y, int largura) {

  char buf[48];
  int len = strlen(txt);
  int esq;
  int i;

  if (largura > 46) largura = 46;
  if (len > largura) len = largura;

  esq = (largura - len) / 2;

  for (i = 0; i < largura; i++) buf[i] = ' ';
  for (i = 0; i < len; i++) buf[esq + i] = txt[i];
  buf[largura] = 0;

  myGLCD.print(buf, (320 - largura * 8) / 2, y);
}

void formatarNumero(char *dst, float v) {
  if (v >= 1000.0)      dtostrf(v, 1, 0, dst);
  else if (v >= 100.0)  dtostrf(v, 1, 1, dst);
  else                  dtostrf(v, 1, 2, dst);
}

void mostrarMensagem(const char *linha1, const char *linha2) {

  myGLCD.clrScr();
  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);
  myGLCD.setFont(SmallFont);

  myGLCD.print(linha1, CENTER, 95);
  myGLCD.print(linha2, CENTER, 120);

  delay(700);
}

// ============================================================
// MENU PRINCIPAL
// ============================================================
void desenharMenuPrincipal() {

  cabecalho("RELE DIGITAL DE PROTECAO");
  myGLCD.print("ARDUINO MEGA 2560 + IHM TOUCH", CENTER, 20);

  desenharBotao(30,  38, 290,  72, AZUL,  "1. SUPERVISAO E OPERACAO");
  desenharBotao(30,  80, 290, 114, VERDE, "2. CONFIGURACAO BASICA");
  desenharBotao(30, 122, 290, 156, AZUL,  "3. CONFIGURACAO AVANCADA");
  desenharBotao(30, 164, 290, 198, VERDE, "4. HISTORICO DE EVENTOS");

  atualizarRodape();
}

void atualizarRodape() {

  if (telaAtual != MENU_PRINCIPAL) return;

  char texto[40];
  word cor;

  if (falhaAtiva) {
    cor = VERMELHO;
    snprintf(texto, sizeof(texto), "FALHA ATUADA: %s", nomeFuncao(funcaoAtuada));
  }
  else {
    cor = CINZA;
    snprintf(texto, sizeof(texto), "%02d/%02d/%04d %02d:%02d:%02d",
             diaAtual, mesAtual, anoAtual,
             horaAtual, minutoAtual, segundoAtual);
  }

  myGLCD.setColor(cor);
  myGLCD.fillRect(10, 206, 310, 232);

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(cor);
  escreverCentro(texto, 213, 36);
  myGLCD.setBackColor(PRETO);
}

void touchMenuPrincipal(int x, int y) {

  // 1. SUPERVISÃO E OPERAÇÃO
  if (dentro(x, y, 30, 38, 290, 72)) {
    irParaTela(SUPERVISAO);
    return;
  }

  // 2. CONFIGURAÇÃO BÁSICA
  if (dentro(x, y, 30, 80, 290, 114)) {
    lerRTC();
    ajDia = diaAtual;
    ajMes = mesAtual;
    ajAno = anoAtual;
    ajHora = horaAtual;
    ajMinuto = minutoAtual;
    ajSegundo = segundoAtual;
    irParaTela(CONFIG_BASICA);
    return;
  }

  // 3. CONFIGURAÇÃO AVANÇADA (pede senha)
  if (dentro(x, y, 30, 122, 290, 156)) {
    abrirTecladoSimples(TECLADO_SENHA, MENU_PRINCIPAL);
    return;
  }

  // 4. HISTÓRICO DE EVENTOS
  if (dentro(x, y, 30, 164, 290, 198)) {
    irParaTela(HISTORICO);
    return;
  }
}

// ============================================================
// 1. SUPERVISÃO E OPERAÇÃO
// ============================================================
void desenharSupervisao() {

  cabecalho("SUPERVISAO E OPERACAO");

  myGLCD.print("TENSOES (V)", 10, 36);
  myGLCD.print("CORRENTES (A)", 10, 68);
  myGLCD.print("DISJUNTOR:", 10, 104);
  myGLCD.print("FUNCAO ATUADA:", 10, 120);

  desenharBotao( 10, 172, 105, 198, VERDE,    "FECHAR");
  desenharBotao(110, 172, 205, 198, VERMELHO, "ABRIR");
  desenharBotao(210, 172, 310, 198, AMARELO,  "RESET", PRETO);
  desenharBotaoVoltar();

  funcaoDesenhada = 255;   // força redesenho das caixas ANSI
  atualizarTelaSupervisao();
}

void atualizarTelaSupervisao() {

  char buf[40];
  char tmp[20];

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);

  // ----------------------------------------------------------
  // DATA / HORA
  // ----------------------------------------------------------
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d  %02d:%02d:%02d",
           diaAtual, mesAtual, anoAtual,
           horaAtual, minutoAtual, segundoAtual);
  myGLCD.print(buf, CENTER, 18);

  // ----------------------------------------------------------
  // TENSÕES
  // ----------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    if (calibTensao[i].kCalculado) formatarNumero(tmp, tensaoRMS[i]);
    else strcpy(tmp, "---");

    snprintf(buf, sizeof(buf), "V%c:%s", 'A' + i, tmp);
    escreverPad(buf, 10 + i * 102, 50, 12);
  }

  // ----------------------------------------------------------
  // CORRENTES
  // ----------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    if (calibCorrente[i].kCalculado) formatarNumero(tmp, correnteRMS[i]);
    else strcpy(tmp, "---");

    snprintf(buf, sizeof(buf), "I%c:%s", 'A' + i, tmp);
    escreverPad(buf, 10 + i * 102, 82, 12);
  }

  // ----------------------------------------------------------
  // ESTADO DO DISJUNTOR
  // ----------------------------------------------------------
  myGLCD.setBackColor(disjuntorFechado ? VERMELHO : VERDE);
  myGLCD.print(disjuntorFechado ? "FECHADO" : "ABERTO ", 100, 104);
  myGLCD.setBackColor(PRETO);

  // ----------------------------------------------------------
  // FUNÇÃO ATUADA (ANSI 50 / 51 / 27 / 59)
  // ----------------------------------------------------------
  if (funcaoDesenhada != funcaoAtuada) {
    funcaoDesenhada = funcaoAtuada;

    const char *nomes[4] = {"ANSI 50", "ANSI 51", "ANSI 27", "ANSI 59"};

    for (int i = 0; i < 4; i++) {
      word cor = (funcaoAtuada == (i + 1)) ? VERMELHO : CINZA;
      desenharBotao(10 + i * 77, 134, 80 + i * 77, 152, cor, nomes[i]);
    }
  }

  // ----------------------------------------------------------
  // INTERTRAVAMENTOS / SUPERVISÃO DE CAMPO
  // ----------------------------------------------------------
  snprintf(buf, sizeof(buf), "MOLA:%s SF6:%s FALHA EXT:%s",
           molaCarregada ? "OK" : "DESC",
           sf6Baixo ? "BAIXO" : "OK",
           falhaGeral ? "SIM" : "NAO");
  escreverPad(buf, 10, 158, 36);
}

void touchSupervisao(int x, int y) {

  if (dentro(x, y, 10, 172, 105, 198)) {
    comandoFechar();
    return;
  }

  if (dentro(x, y, 110, 172, 205, 198)) {
    comandoAbrir();
    return;
  }

  if (dentro(x, y, 210, 172, 310, 198)) {
    comandoReset();
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_PRINCIPAL);
    return;
  }
}

// ============================================================
// COMANDOS DO DISJUNTOR
// ============================================================
void comandoFechar() {

  const char *motivo = 0;

  if (disjuntorFechado)    motivo = "DISJUNTOR JA FECHADO";
  else if (falhaAtiva)     motivo = "FALHA ATIVA - FACA RESET";
  else if (!molaCarregada) motivo = "MOLA DESCARREGADA";
  else if (sf6Baixo)       motivo = "SF6 MUITO BAIXO";
  else if (falhaGeral)     motivo = "FALHA GERAL ATIVA";
  else if (pulsoTrip)      motivo = "TRIP EM ANDAMENTO";

  if (motivo != 0) {
    mostrarMensagem("FECHAMENTO BLOQUEADO", motivo);
    desenharSupervisao();
    return;
  }

  acionarFechamento();
  Serial.println(">>> COMANDO FECHAR DISJUNTOR");
}

void comandoAbrir() {

  if (!disjuntorFechado) {
    mostrarMensagem("DISJUNTOR JA ABERTO", "COMANDO IGNORADO");
    desenharSupervisao();
    return;
  }

  acionarTrip();
  Serial.println(">>> COMANDO ABRIR DISJUNTOR");
}

void comandoReset() {

  falhaAtiva = false;
  funcaoAtuada = FUNC_NENHUMA;
  resetarTemporizadores();

  Serial.println(">>> RESET DE FALHA");

  mostrarMensagem("RESET DE FALHA", "INDICACOES LIMPAS");
  desenharSupervisao();
}

// ============================================================
// SAÍDAS DIGITAIS (pulsos não bloqueantes)
// ============================================================
void acionarTrip() {

  // Abertura tem prioridade: cancela fechamento pendente
  digitalWrite(PINO_FECHAR, LOW);
  pulsoFecha = false;

  digitalWrite(PINO_TRIP, HIGH);
  pulsoTrip = true;
  inicioPulsoTrip = millis();
}

void acionarFechamento() {

  digitalWrite(PINO_FECHAR, HIGH);
  pulsoFecha = true;
  inicioPulsoFecha = millis();
}

void gerenciarSaidas() {

  if (pulsoTrip && (millis() - inicioPulsoTrip >= PULSO_COMANDO_MS)) {
    digitalWrite(PINO_TRIP, LOW);
    pulsoTrip = false;
  }

  if (pulsoFecha && (millis() - inicioPulsoFecha >= PULSO_COMANDO_MS)) {
    digitalWrite(PINO_FECHAR, LOW);
    pulsoFecha = false;
  }
}

// ============================================================
// ENTRADAS DIGITAIS
// ============================================================
bool lerEntrada(int pino) {
  bool nivel = (digitalRead(pino) == HIGH);
  return ENTRADAS_ATIVAS_EM_BAIXO ? !nivel : nivel;
}

void lerEntradas() {
  disjuntorFechado = lerEntrada(PINO_ESTADO_DISJUNTOR);
  molaCarregada    = lerEntrada(PINO_MOLA);
  sf6Baixo         = lerEntrada(PINO_SF6);
  falhaGeral       = lerEntrada(PINO_FALHAS);
}

// ============================================================
// 2. CONFIGURAÇÃO BÁSICA
// ============================================================
void desenharConfigBasica() {

  cabecalho("CONFIGURACAO BASICA");

  myGLCD.print("RTC DS3231:", 10, 24);
  atualizarRelogioConfig();

  myGLCD.print("VALORES A GRAVAR", 10, 52);

  char buf[40];

  snprintf(buf, sizeof(buf), "DATA: %02d/%02d/%04d", ajDia, ajMes, ajAno);
  myGLCD.print(buf, 10, 70);

  snprintf(buf, sizeof(buf), "HORA: %02d:%02d:%02d", ajHora, ajMinuto, ajSegundo);
  myGLCD.print(buf, 10, 86);

  desenharBotao( 10, 106, 155, 140, AZUL,  "AJUSTAR DATA");
  desenharBotao(165, 106, 310, 140, VERDE, "AJUSTAR HORA");
  desenharBotao( 30, 150, 290, 184, AMARELO, "ATUALIZAR RTC DS3231", PRETO);
  desenharBotaoVoltar();
}

void atualizarRelogioConfig() {

  if (telaAtual != CONFIG_BASICA) return;

  char buf[40];

  snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d",
           diaAtual, mesAtual, anoAtual,
           horaAtual, minutoAtual, segundoAtual);

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);
  myGLCD.print(buf, 110, 24);
}

void touchConfigBasica(int x, int y) {

  // AJUSTAR DATA (Dia / Mês / Ano)
  if (dentro(x, y, 10, 106, 155, 140)) {
    abrirTecladoSimples(TECLADO_DATA, CONFIG_BASICA);
    return;
  }

  // AJUSTAR HORA (Hora / Minuto / Segundo)
  if (dentro(x, y, 165, 106, 310, 140)) {
    abrirTecladoSimples(TECLADO_HORA, CONFIG_BASICA);
    return;
  }

  // ATUALIZAR RTC DS3231
  if (dentro(x, y, 30, 150, 290, 184)) {
    gravarRTC();
    mostrarMensagem("RTC DS3231", "ATUALIZADO");
    lerRTC();
    desenharConfigBasica();
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_PRINCIPAL);
    return;
  }
}

void gravarRTC() {

  rtc.adjust(DateTime(ajAno, ajMes, ajDia, ajHora, ajMinuto, ajSegundo));
  lerRTC();

  Serial.println("--------------------------------");
  Serial.println("DATA/HORA SALVA NO DS3231");
  Serial.println("--------------------------------");
}

void lerRTC() {
  agoraRTC = rtc.now();
  diaAtual = agoraRTC.day();
  mesAtual = agoraRTC.month();
  anoAtual = agoraRTC.year();
  horaAtual = agoraRTC.hour();
  minutoAtual = agoraRTC.minute();
  segundoAtual = agoraRTC.second();
}

int diasNoMes(int m, int a) {

  static const int dias[12] = {31, 28, 31, 30, 31, 30,
                               31, 31, 30, 31, 30, 31};

  if (m < 1 || m > 12) return 0;

  if (m == 2) {
    bool bissexto = ((a % 4 == 0) && (a % 100 != 0)) || (a % 400 == 0);
    return bissexto ? 29 : 28;
  }

  return dias[m - 1];
}

bool validarData(int d, int m, int a) {
  if (a < 2000 || a > 2099) return false;
  if (m < 1 || m > 12) return false;
  if (d < 1 || d > diasNoMes(m, a)) return false;
  return true;
}

bool validarHora(int h, int mi, int s) {
  return (h >= 0 && h <= 23 &&
          mi >= 0 && mi <= 59 &&
          s >= 0 && s <= 59);
}

// ============================================================
// TECLADO NUMÉRICO GENÉRICO
// (data, hora, senha e valores numéricos de ajuste)
// ============================================================
void abrirTecladoSimples(ModoTeclado modo, Tela retorno) {
  modoTeclado = modo;
  telaRetorno = retorno;
  entradaTeclado = "";
  irParaTela(TECLADO);
}

void abrirTecladoNumero(Parametro p, const char *titulo,
                        const char *unidade, Tela retorno) {
  modoTeclado = TECLADO_NUMERO;
  parametroTeclado = p;
  tituloTeclado = titulo;
  unidadeTeclado = unidade;
  telaRetorno = retorno;
  entradaTeclado = "";
  irParaTela(TECLADO);
}

int maxDigitosTeclado() {
  switch (modoTeclado) {
    case TECLADO_DATA:  return 8;
    case TECLADO_HORA:  return 6;
    case TECLADO_SENHA: return 4;
    default:            return 8;
  }
}

void desenharTeclado() {

  const char *l1 = "";
  const char *l2 = "";

  cabecalho("");

  switch (modoTeclado) {
    case TECLADO_DATA:
      l1 = "INSIRA A DATA";
      l2 = "DDMMAAAA";
      break;
    case TECLADO_HORA:
      l1 = "INSIRA A HORA";
      l2 = "HHMMSS";
      break;
    case TECLADO_SENHA:
      l1 = "SENHA DE ACESSO";
      l2 = "4 DIGITOS";
      break;
    case TECLADO_NUMERO:
      l1 = tituloTeclado;
      l2 = unidadeTeclado;
      break;
  }

  myGLCD.print(l1, CENTER, 5);
  myGLCD.print(l2, CENTER, 20);

  atualizarCampoTeclado();

  // Dígitos 1..9
  char rotulo[2] = {0, 0};

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      rotulo[0] = '1' + r * 3 + c;
      desenharBotao(TEC_X[c], TEC_Y[r],
                    TEC_X[c] + TEC_W, TEC_Y[r] + TEC_H,
                    AZUL, rotulo);
    }
  }

  // Ponto decimal (somente valores numéricos)
  if (modoTeclado == TECLADO_NUMERO) {
    desenharBotao(TEC_X[0], TEC_Y[3],
                  TEC_X[0] + TEC_W, TEC_Y[3] + TEC_H, AZUL, ".");
  }

  // Zero
  desenharBotao(TEC_X[1], TEC_Y[3],
                TEC_X[1] + TEC_W, TEC_Y[3] + TEC_H, AZUL, "0");

  // Ações
  desenharBotao(240,  62, 310,  96, VERMELHO, "APAGAR");
  desenharBotao(240, 102, 310, 136, CINZA,    "CANCEL");
  desenharBotao(240, 142, 310, 216, VERDE,    "OK");
}

void atualizarCampoTeclado() {

  char txt[16];
  int n = entradaTeclado.length();

  if (n > 15) n = 15;

  for (int i = 0; i < n; i++) {
    txt[i] = (modoTeclado == TECLADO_SENHA) ? '*' : entradaTeclado[i];
  }
  txt[n] = 0;

  myGLCD.setColor(CINZA);
  myGLCD.fillRect(25, 36, 295, 58);

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(CINZA);
  myGLCD.print(txt, CENTER, 41);
  myGLCD.setBackColor(PRETO);
}

void touchTeclado(int x, int y) {

  // Dígitos 1..9
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      if (dentro(x, y, TEC_X[c], TEC_Y[r],
                 TEC_X[c] + TEC_W, TEC_Y[r] + TEC_H)) {
        processarTeclaNumerica('1' + r * 3 + c);
        return;
      }
    }
  }

  // Zero
  if (dentro(x, y, TEC_X[1], TEC_Y[3],
             TEC_X[1] + TEC_W, TEC_Y[3] + TEC_H)) {
    processarTeclaNumerica('0');
    return;
  }

  // Ponto
  if (modoTeclado == TECLADO_NUMERO &&
      dentro(x, y, TEC_X[0], TEC_Y[3],
             TEC_X[0] + TEC_W, TEC_Y[3] + TEC_H)) {
    processarTeclaNumerica('.');
    return;
  }

  // APAGAR
  if (dentro(x, y, 240, 62, 310, 96)) {
    if (entradaTeclado.length() > 0) {
      entradaTeclado.remove(entradaTeclado.length() - 1);
      atualizarCampoTeclado();
    }
    return;
  }

  // CANCELAR
  if (dentro(x, y, 240, 102, 310, 136)) {
    entradaTeclado = "";
    irParaTela(telaRetorno);
    return;
  }

  // OK
  if (dentro(x, y, 240, 142, 310, 216)) {
    confirmarTeclado();
    return;
  }
}

void processarTeclaNumerica(char tecla) {

  if ((int)entradaTeclado.length() >= maxDigitosTeclado()) return;

  if (tecla == '.') {
    if (entradaTeclado.length() == 0) return;
    if (entradaTeclado.indexOf('.') >= 0) return;
  }

  entradaTeclado += tecla;
  atualizarCampoTeclado();
}

void confirmarTeclado() {

  // ----------------------------------------------------------
  // DATA
  // ----------------------------------------------------------
  if (modoTeclado == TECLADO_DATA) {

    if (entradaTeclado.length() != 8) {
      mostrarMensagem("DATA INCOMPLETA", "DIGITE DDMMAAAA");
      desenharTeclado();
      return;
    }

    int dia = entradaTeclado.substring(0, 2).toInt();
    int mes = entradaTeclado.substring(2, 4).toInt();
    int ano = entradaTeclado.substring(4, 8).toInt();

    if (!validarData(dia, mes, ano)) {
      mostrarMensagem("DATA INVALIDA", "VERIFIQUE OS VALORES");
      desenharTeclado();
      return;
    }

    ajDia = dia;
    ajMes = mes;
    ajAno = ano;

    entradaTeclado = "";
    irParaTela(telaRetorno);
    return;
  }

  // ----------------------------------------------------------
  // HORA
  // ----------------------------------------------------------
  if (modoTeclado == TECLADO_HORA) {

    if (entradaTeclado.length() != 6) {
      mostrarMensagem("HORA INCOMPLETA", "DIGITE HHMMSS");
      desenharTeclado();
      return;
    }

    int hora = entradaTeclado.substring(0, 2).toInt();
    int minuto = entradaTeclado.substring(2, 4).toInt();
    int segundo = entradaTeclado.substring(4, 6).toInt();

    if (!validarHora(hora, minuto, segundo)) {
      mostrarMensagem("HORA INVALIDA", "VERIFIQUE OS VALORES");
      desenharTeclado();
      return;
    }

    ajHora = hora;
    ajMinuto = minuto;
    ajSegundo = segundo;

    entradaTeclado = "";
    irParaTela(telaRetorno);
    return;
  }

  // ----------------------------------------------------------
  // SENHA
  // ----------------------------------------------------------
  if (modoTeclado == TECLADO_SENHA) {

    if (entradaTeclado == String(SENHA_AVANCADA)) {
      entradaTeclado = "";
      irParaTela(MENU_AVANCADO);
    }
    else {
      mostrarMensagem("SENHA INCORRETA", "TENTE NOVAMENTE");
      entradaTeclado = "";
      desenharTeclado();
    }
    return;
  }

  // ----------------------------------------------------------
  // VALOR NUMÉRICO DE AJUSTE
  // ----------------------------------------------------------
  float valor = entradaTeclado.toFloat();

  if (!aplicarParametro(parametroTeclado, valor)) {
    mostrarMensagem("VALOR INVALIDO", "VERIFIQUE O AJUSTE");
    desenharTeclado();
    return;
  }

  salvarConfigEEPROM();
  resetarTemporizadores();

  entradaTeclado = "";
  irParaTela(telaRetorno);
}

bool aplicarParametro(Parametro p, float valor) {

  switch (p) {

    case PAR_TC:
      if (valor < 5.0 || valor > 20000.0) return false;
      cfg.tcPrimario = valor;
      return true;

    case PAR_TP:
      if (valor < 115.0 || valor > 500000.0) return false;
      cfg.tpPrimario = valor;
      return true;

    case PAR_50:
      if (valor <= 0.0 || valor > 100000.0) return false;
      cfg.pickup50 = valor;
      return true;

    case PAR_51_PICKUP:
      if (valor <= 0.0 || valor > 100000.0) return false;
      cfg.pickup51 = valor;
      return true;

    case PAR_51_TMS:
      if (valor < 0.01 || valor > 10.0) return false;
      cfg.tms51 = valor;
      return true;

    case PAR_27:
      if (valor <= 0.0 || valor >= 100.0) return false;
      cfg.pct27 = valor;
      return true;

    case PAR_59:
      if (valor <= 100.0 || valor > 200.0) return false;
      cfg.pct59 = valor;
      return true;
  }

  return false;
}

// ============================================================
// 3. CONFIGURAÇÃO AVANÇADA - MENU
// ============================================================
const char *rotulosAvancado[7] = {
  "CALIBRACAO", "RELACAO TCs", "RELACAO TPs",
  "ANSI 50", "ANSI 51", "ANSI 27", "ANSI 59"
};

const Tela destinosAvancado[7] = {
  MENU_CALIBRACAO, TELA_TC, TELA_TP,
  TELA_ANSI50, TELA_ANSI51, TELA_ANSI27, TELA_ANSI59
};

void desenharMenuAvancado() {

  cabecalho("CONFIGURACAO AVANCADA");

  for (int i = 0; i < 7; i++) {
    int col = i % 2;
    int lin = i / 2;
    int x1 = (col == 0) ? 10 : 165;
    int y1 = 28 + lin * 44;

    desenharBotao(x1, y1, x1 + 145, y1 + 36,
                  (i % 3 == 0) ? AZUL : VERDE, rotulosAvancado[i]);
  }

  desenharBotaoVoltar();
}

void touchMenuAvancado(int x, int y) {

  for (int i = 0; i < 7; i++) {
    int col = i % 2;
    int lin = i / 2;
    int x1 = (col == 0) ? 10 : 165;
    int y1 = 28 + lin * 44;

    if (dentro(x, y, x1, y1, x1 + 145, y1 + 36)) {
      irParaTela(destinosAvancado[i]);
      return;
    }
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_PRINCIPAL);
    return;
  }
}

// ============================================================
// CALIBRAÇÃO DOS SENSORES
// ============================================================
void desenharMenuCalibracao() {

  cabecalho("CALIBRACAO DOS SENSORES");

  myGLCD.print("TENSAO (ZMPT101B): A0 A1 A2", 10, 26);

  desenharBotao( 10, 40, 100, 80, AZUL, "VA");
  desenharBotao(110, 40, 210, 80, AZUL, "VB");
  desenharBotao(220, 40, 310, 80, AZUL, "VC");

  myGLCD.print("CORRENTE (ACS712): A3 A4 A5", 10, 92);

  desenharBotao( 10, 106, 100, 146, VERDE, "IA");
  desenharBotao(110, 106, 210, 146, VERDE, "IB");
  desenharBotao(220, 106, 310, 146, VERDE, "IC");

  myGLCD.print("REF: 115 V / 5 A  (SEM PROTECAO)", CENTER, 165);

  desenharBotaoVoltar();
}

void touchMenuCalibracao(int x, int y) {

  int idx = -1;

  if (dentro(x, y,  10,  40, 100,  80)) idx = 0;
  if (dentro(x, y, 110,  40, 210,  80)) idx = 1;
  if (dentro(x, y, 220,  40, 310,  80)) idx = 2;
  if (dentro(x, y,  10, 106, 100, 146)) idx = 3;
  if (dentro(x, y, 110, 106, 210, 146)) idx = 4;
  if (dentro(x, y, 220, 106, 310, 146)) idx = 5;

  if (idx >= 0) {
    canalCal = idx;
    irParaTela(TELA_CALIBRACAO);
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_AVANCADO);
    return;
  }
}

Calibracao &calibAtual() {
  return (canalCal < 3) ? calibTensao[canalCal] : calibCorrente[canalCal - 3];
}

int pinoCalAtual() {
  return (canalCal < 3) ? pinosZMPT[canalCal] : pinosACS[canalCal - 3];
}

void desenharTelaCalibracao() {

  bool ehTensao = (canalCal < 3);
  char fase = 'A' + (canalCal % 3);
  char buf[40];
  char tmp[20];

  snprintf(buf, sizeof(buf), "CALIBRACAO %s FASE %c",
           ehTensao ? "TENSAO" : "CORRENTE", fase);
  cabecalho(buf);

  myGLCD.print("RMS ATUAL:", 10, 22);
  atualizarLiveCalibracao();

  // ----------------------------------------------------------
  // PASSOS
  // ----------------------------------------------------------
  desenharBotao(10, 36, 170, 62, AZUL, "SALVAR OFFSET");
  myGLCD.print(ehTensao ? "APLIQUE 0 V" : "APLIQUE 0 A", 178, 43);

  desenharBotao(10, 66, 170, 92, VERDE, "SALVAR REFERENCIA");
  myGLCD.print(ehTensao ? "APLIQUE 115 V" : "APLIQUE 5 A", 178, 73);

  desenharBotao(10, 96, 170, 122, AMARELO, "CALCULAR K", PRETO);

  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------
  Calibracao &c = calibAtual();

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);

  myGLCD.print("OFFSET:", 10, 132);
  if (c.zeroCalibrado) dtostrf(c.zeroRMS, 1, 4, tmp); else strcpy(tmp, "---");
  myGLCD.print(tmp, 100, 132);

  myGLCD.print("REF:", 10, 148);
  if (c.referenciaCalibrada) dtostrf(c.referenciaRMS, 1, 4, tmp); else strcpy(tmp, "---");
  myGLCD.print(tmp, 100, 148);

  myGLCD.print("K:", 10, 164);
  if (c.kCalculado) dtostrf(c.fatorK, 1, 4, tmp); else strcpy(tmp, "---");
  myGLCD.print(tmp, 100, 164);

  myGLCD.setColor(AMARELO);
  myGLCD.print("PROTECOES INIBIDAS", CENTER, 184);
  myGLCD.setColor(BRANCO);

  desenharBotaoVoltar();
}

void atualizarLiveCalibracao() {

  char tmp[20];
  char buf[24];

  float rms = lerRMS(pinoCalAtual());
  dtostrf(rms, 1, 4, tmp);
  snprintf(buf, sizeof(buf), "%s", tmp);

  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);
  escreverPad(buf, 100, 22, 12);
}

void touchTelaCalibracao(int x, int y) {

  if (dentro(x, y, 10, 36, 170, 62)) {
    calibrarOffset();
    desenharTelaCalibracao();
    return;
  }

  if (dentro(x, y, 10, 66, 170, 92)) {
    calibrarReferencia();
    desenharTelaCalibracao();
    return;
  }

  if (dentro(x, y, 10, 96, 170, 122)) {
    calcularK();
    desenharTelaCalibracao();
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_CALIBRACAO);
    return;
  }
}

void calibrarOffset() {

  mostrarMensagem("SALVANDO OFFSET", "AGUARDE...");

  Calibracao &c = calibAtual();

  c.zeroRMS = lerRMSMedio(pinoCalAtual());
  c.zeroCalibrado = true;
  c.kCalculado = false;       // exige novo cálculo de K

  Serial.print("OFFSET = ");
  Serial.println(c.zeroRMS, 5);

  salvarCalibracaoEEPROM();
}

void calibrarReferencia() {

  mostrarMensagem("SALVANDO REFERENCIA", "AGUARDE...");

  Calibracao &c = calibAtual();

  c.referenciaRMS = lerRMSMedio(pinoCalAtual());
  c.valorReferencia = (canalCal < 3) ? TENSAO_REFERENCIA : CORRENTE_REFERENCIA;
  c.referenciaCalibrada = true;
  c.kCalculado = false;       // exige novo cálculo de K

  Serial.print("REFERENCIA = ");
  Serial.println(c.referenciaRMS, 5);

  salvarCalibracaoEEPROM();
}

void calcularK() {

  Calibracao &c = calibAtual();

  if (!c.zeroCalibrado || !c.referenciaCalibrada) {
    mostrarMensagem("CALCULO DE K", "SALVE OFFSET E REF");
    return;
  }

  if (c.referenciaRMS <= c.zeroRMS) {
    mostrarMensagem("ERRO NA CALIBRACAO", "REF <= OFFSET");
    return;
  }

  c.fatorK = c.valorReferencia / (c.referenciaRMS - c.zeroRMS);
  c.kCalculado = true;

  Serial.print("K = ");
  Serial.println(c.fatorK, 5);

  salvarCalibracaoEEPROM();

  mostrarMensagem("K CALCULADO", "CALIBRACAO SALVA");
}

// ============================================================
// RELAÇÃO DOS TCs / TPs
// ============================================================
float relacaoTC() {
  return cfg.tcPrimario / TC_SECUNDARIO;
}

float relacaoTP() {
  return cfg.tpPrimario / TP_SECUNDARIO;
}

void desenharRelacao(bool ehTC) {

  char buf[40];
  char tmp[20];

  cabecalho(ehTC ? "RELACAO DOS TCs" : "RELACAO DOS TPs");

  myGLCD.print(ehTC ? "CORRENTE PRIMARIA:" : "TENSAO PRIMARIA:", 10, 36);

  dtostrf(ehTC ? cfg.tcPrimario : cfg.tpPrimario, 1, 1, tmp);
  snprintf(buf, sizeof(buf), "%s %s", tmp, ehTC ? "A" : "V");
  myGLCD.print(buf, 10, 52);

  desenharBotao(200, 40, 310, 66, AZUL, "AJUSTAR");

  myGLCD.print(ehTC ? "CORRENTE SECUNDARIA = 5 A"
                    : "TENSAO SECUNDARIA = 115 V", 10, 84);

  myGLCD.print("EQUACAO:", 10, 108);
  myGLCD.print(ehTC ? "RTC = Iprimario / 5"
                    : "RTP = Vprimario / 115", 10, 124);
  myGLCD.print(ehTC ? "Iprimaria = Isecundaria x RTC"
                    : "Vprimaria = Vsecundaria x RTP", 10, 140);

  dtostrf(ehTC ? relacaoTC() : relacaoTP(), 1, 3, tmp);
  snprintf(buf, sizeof(buf), "%s = %s", ehTC ? "RTC" : "RTP", tmp);
  myGLCD.print(buf, 10, 164);

  desenharBotaoVoltar();
}

void touchRelacao(int x, int y, bool ehTC) {

  if (dentro(x, y, 200, 40, 310, 66)) {
    if (ehTC) {
      abrirTecladoNumero(PAR_TC, "CORRENTE PRIMARIA TC", "A", TELA_TC);
    }
    else {
      abrirTecladoNumero(PAR_TP, "TENSAO PRIMARIA TP", "V", TELA_TP);
    }
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_AVANCADO);
    return;
  }
}

// ============================================================
// ANSI 50 - SOBRECORRENTE INSTANTÂNEA
// ============================================================
void desenharAnsi50() {

  char buf[40];
  char tmp[20];

  cabecalho("ANSI 50 - SOBRECORRENTE INST.");

  myGLCD.print("PICKUP DE CORRENTE (PRIMARIO):", 10, 34);

  dtostrf(cfg.pickup50, 1, 2, tmp);
  snprintf(buf, sizeof(buf), "%s A", tmp);
  myGLCD.print(buf, 10, 50);

  myGLCD.print("EQUACAO 01:", 10, 80);
  myGLCD.print("Imedida >= Ipickup", 10, 96);
  myGLCD.print("SE VERDADEIRO => TRIP DISJUNTOR", 10, 112);

  desenharBotao(30, 150, 290, 184, AZUL, "AJUSTAR PICKUP");
  desenharBotaoVoltar();
}

void touchAnsi50(int x, int y) {

  if (dentro(x, y, 30, 150, 290, 184)) {
    abrirTecladoNumero(PAR_50, "PICKUP ANSI 50", "A (PRIMARIO)", TELA_ANSI50);
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_AVANCADO);
    return;
  }
}

// ============================================================
// ANSI 51 - SOBRECORRENTE TEMPORIZADA
// ============================================================
void desenharAnsi51() {

  char buf[40];
  char tmp[20];

  cabecalho("ANSI 51 - SOBRECORR. TEMPORIZADA");

  // Curva IEC
  desenharBotao( 10, 24, 155, 50,
                 (cfg.curva51 == CURVA_SI) ? VERDE : CINZA, "STD INVERSE (SI)");
  desenharBotao(165, 24, 310, 50,
                 (cfg.curva51 == CURVA_EI) ? VERDE : CINZA, "EXT INVERSE (EI)");

  // Pickup
  dtostrf(cfg.pickup51, 1, 2, tmp);
  snprintf(buf, sizeof(buf), "PICKUP: %s A", tmp);
  myGLCD.setColor(BRANCO);
  myGLCD.setBackColor(PRETO);
  myGLCD.print(buf, 10, 64);
  desenharBotao(200, 56, 310, 82, AZUL, "AJUSTAR");

  // TMS
  dtostrf(cfg.tms51, 1, 2, tmp);
  snprintf(buf, sizeof(buf), "TMS: %s", tmp);
  myGLCD.print(buf, 10, 96);
  desenharBotao(200, 88, 310, 114, AZUL, "AJUSTAR");

  // Equações
  myGLCD.print("EQUACAO 02: M = Imedida/Ipickup", 10, 122);

  if (cfg.curva51 == CURVA_SI) {
    myGLCD.print("EQ 03: t = 0,14 x TMS/(M^0,02-1)", 10, 138);
  }
  else {
    myGLCD.print("EQ 04: t = 80 x TMS/(M^2 - 1)", 10, 138);
  }

  myGLCD.print("SE M > 1 INICIA TEMPORIZACAO", 10, 154);
  myGLCD.print("SE TEMPO >= t => TRIP DISJUNTOR", 10, 170);

  desenharBotaoVoltar();
}

void touchAnsi51(int x, int y) {

  if (dentro(x, y, 10, 24, 155, 50)) {
    cfg.curva51 = CURVA_SI;
    salvarConfigEEPROM();
    resetarTemporizadores();
    desenharAnsi51();
    return;
  }

  if (dentro(x, y, 165, 24, 310, 50)) {
    cfg.curva51 = CURVA_EI;
    salvarConfigEEPROM();
    resetarTemporizadores();
    desenharAnsi51();
    return;
  }

  if (dentro(x, y, 200, 56, 310, 82)) {
    abrirTecladoNumero(PAR_51_PICKUP, "PICKUP ANSI 51", "A (PRIMARIO)", TELA_ANSI51);
    return;
  }

  if (dentro(x, y, 200, 88, 310, 114)) {
    abrirTecladoNumero(PAR_51_TMS, "TMS ANSI 51", "0.01 A 10", TELA_ANSI51);
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_AVANCADO);
    return;
  }
}

// ============================================================
// ANSI 27 / ANSI 59 - SUBTENSÃO / SOBRETENSÃO
// ============================================================
void desenharAnsiTensao(bool subtensao) {

  char buf[44];
  char tmp[20];

  cabecalho(subtensao ? "ANSI 27 - SUBTENSAO" : "ANSI 59 - SOBRETENSAO");

  float pct = subtensao ? cfg.pct27 : cfg.pct59;
  float vNominal = cfg.tpPrimario;               // 115 V x RTP
  float vLimite = (pct * vNominal) / 100.0;      // Eq. 05 / Eq. 07

  myGLCD.print("PICKUP DE TENSAO (%):", 10, 30);

  dtostrf(pct, 1, 1, tmp);
  snprintf(buf, sizeof(buf), "%s %%", tmp);
  myGLCD.print(buf, 10, 46);

  snprintf(buf, sizeof(buf), "EQUACAO %s:", subtensao ? "05" : "07");
  myGLCD.print(buf, 10, 66);
  myGLCD.print("Vlimite = (Ppickup x Vnominal)/100", 10, 80);

  dtostrf(vNominal, 1, 1, tmp);
  snprintf(buf, sizeof(buf), "Vnominal = %s V", tmp);
  myGLCD.print(buf, 10, 96);

  dtostrf(vLimite, 1, 1, tmp);
  snprintf(buf, sizeof(buf), "Vlimite  = %s V", tmp);
  myGLCD.print(buf, 10, 110);

  snprintf(buf, sizeof(buf), "EQUACAO %s:", subtensao ? "06" : "08");
  myGLCD.print(buf, 10, 130);
  myGLCD.print(subtensao ? "Vmedida < Vlimite" : "Vmedida > Vlimite", 10, 144);
  myGLCD.print("SE VERDADEIRO => TRIP DISJUNTOR", 10, 158);

  desenharBotao(30, 174, 290, 200, AZUL, "AJUSTAR PICKUP");
  desenharBotaoVoltar();
}

void touchAnsiTensao(int x, int y, bool subtensao) {

  if (dentro(x, y, 30, 174, 290, 200)) {
    if (subtensao) {
      abrirTecladoNumero(PAR_27, "PICKUP ANSI 27", "% DA NOMINAL", TELA_ANSI27);
    }
    else {
      abrirTecladoNumero(PAR_59, "PICKUP ANSI 59", "% DA NOMINAL", TELA_ANSI59);
    }
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_AVANCADO);
    return;
  }
}

// ============================================================
// 4. HISTÓRICO DE EVENTOS
// ============================================================
void montarListaHistorico() {

  qtdHist = 0;

  uint32_t agora = rtc.now().unixtime();

  // Do mais recente (k = 0) para o mais antigo
  for (int k = 0; k < hist.total && k < MAX_EVENTOS; k++) {

    int pos = ((int)hist.proximo - 1 - k + MAX_EVENTOS) % MAX_EVENTOS;
    uint32_t t = hist.ev[pos].instante;

    bool dentroJanela = (agora < t) || ((agora - t) <= JANELA_HISTORICO_S);

    if (dentroJanela) {
      idxHist[qtdHist++] = pos;
    }
  }
}

void desenharHistorico() {

  char buf[44];
  char tmp[20];

  cabecalho("HISTORICO DE EVENTOS");

  if (qtdHist == 0) {
    myGLCD.print("NENHUM EVENTO NAS", CENTER, 90);
    myGLCD.print("ULTIMAS 48 HORAS", CENTER, 108);
    desenharBotaoVoltar();
    return;
  }

  if (eventoSel >= qtdHist) eventoSel = qtdHist - 1;
  if (eventoSel < 0) eventoSel = 0;

  Evento &e = hist.ev[idxHist[eventoSel]];
  DateTime dt(e.instante);

  snprintf(buf, sizeof(buf), "EVENTO %d/%d%s", eventoSel + 1, qtdHist,
           (eventoSel == 0) ? " (MAIS RECENTE)" : "");
  myGLCD.print(buf, 10, 22);

  snprintf(buf, sizeof(buf), "DATA: %02d/%02d/%04d  HORA: %02d:%02d:%02d",
           dt.day(), dt.month(), dt.year(),
           dt.hour(), dt.minute(), dt.second());
  myGLCD.print(buf, 10, 38);

  snprintf(buf, sizeof(buf), "FUNCAO ATUADA: %s", nomeFuncao(e.funcao));
  myGLCD.print(buf, 10, 54);

  myGLCD.print("TENSOES REGISTRADAS (V)", 10, 74);
  for (int i = 0; i < 3; i++) {
    formatarNumero(tmp, e.v[i]);
    snprintf(buf, sizeof(buf), "V%c:%s", 'A' + i, tmp);
    escreverPad(buf, 10 + i * 102, 88, 12);
  }

  myGLCD.print("CORRENTES REGISTRADAS (A)", 10, 108);
  for (int i = 0; i < 3; i++) {
    formatarNumero(tmp, e.i[i]);
    snprintf(buf, sizeof(buf), "I%c:%s", 'A' + i, tmp);
    escreverPad(buf, 10 + i * 102, 122, 12);
  }

  snprintf(buf, sizeof(buf), "ESTADO DO DISJUNTOR: %s",
           e.estadoDisjuntor ? "FECHADO" : "ABERTO");
  myGLCD.print(buf, 10, 146);

  desenharBotao( 10, 172, 105, 198, AZUL, "ANTERIOR");
  desenharBotao(215, 172, 310, 198, AZUL, "PROXIMO");
  desenharBotaoVoltar();
}

void touchHistorico(int x, int y) {

  if (dentro(x, y, 10, 172, 105, 198)) {
    if (eventoSel > 0) {
      eventoSel--;
      desenharHistorico();
    }
    return;
  }

  if (dentro(x, y, 215, 172, 310, 198)) {
    if (eventoSel < qtdHist - 1) {
      eventoSel++;
      desenharHistorico();
    }
    return;
  }

  if (tocouVoltar(x, y)) {
    irParaTela(MENU_PRINCIPAL);
    return;
  }
}

Evento montarEvento(uint8_t funcao) {

  Evento e;

  e.instante = rtc.now().unixtime();
  e.funcao = funcao;
  e.estadoDisjuntor = disjuntorFechado ? 1 : 0;

  for (int i = 0; i < 3; i++) {
    e.v[i] = tensaoRMS[i];
    e.i[i] = correnteRMS[i];
  }

  return e;
}

void gravarEvento(const Evento &e) {

  hist.ev[hist.proximo] = e;
  hist.proximo = (hist.proximo + 1) % MAX_EVENTOS;

  if (hist.total < MAX_EVENTOS) hist.total++;

  salvarHistoricoEEPROM();
}

const char *nomeFuncao(uint8_t f) {
  switch (f) {
    case FUNC_ANSI50: return "ANSI 50";
    case FUNC_ANSI51: return "ANSI 51";
    case FUNC_ANSI27: return "ANSI 27";
    case FUNC_ANSI59: return "ANSI 59";
    default:          return "NENHUMA";
  }
}

// ============================================================
// MEDIÇÃO RMS
// ============================================================
// Amostra durante 1 ciclo completo da rede (janela de tempo),
// calculando média e variância em uma única passagem.
float lerRMS(int pino) {

  long soma = 0;
  long somaQuad = 0;
  unsigned int n = 0;

  analogRead(pino);   // descarta 1ª leitura (estabilização do MUX)

  unsigned long t0 = micros();

  while (micros() - t0 < JANELA_AMOSTRAGEM_US) {
    int leitura = analogRead(pino);
    soma += leitura;
    somaQuad += (long)leitura * (long)leitura;
    n++;
  }

  if (n == 0) return 0;

  float media = (float)soma / n;
  float variancia = ((float)somaQuad / n) - (media * media);

  if (variancia < 0) variancia = 0;

  return sqrt(variancia) * VREF_ADC / ADC_MAX;
}

float lerRMSMedio(int pino) {

  float acc = 0;

  for (int i = 0; i < AMOSTRAS_CALIBRACAO; i++) {
    acc += lerRMS(pino);
  }

  return acc / AMOSTRAS_CALIBRACAO;
}

// Mede um canal por chamada (V A, V B, V C, I A, I B, I C)
void medirProximoCanal() {

  int c = canalMedicao;

  if (c < 3) {
    int i = c;
    Calibracao &cal = calibTensao[i];

    if (cal.zeroCalibrado && cal.kCalculado) {
      float rms = lerRMS(pinosZMPT[i]);
      float v = (rms - cal.zeroRMS) * cal.fatorK;
      if (v < 0) v = 0;

      tensaoSec[i] = v;                    // secundário
      tensaoRMS[i] = v * relacaoTP();      // primário (RTP)
    }
    else {
      tensaoSec[i] = 0;
      tensaoRMS[i] = 0;
    }
  }
  else {
    int i = c - 3;
    Calibracao &cal = calibCorrente[i];

    if (cal.zeroCalibrado && cal.kCalculado) {
      float rms = lerRMS(pinosACS[i]);
      float a = (rms - cal.zeroRMS) * cal.fatorK;
      if (a < 0) a = 0;

      correnteSec[i] = a;                  // secundário
      correnteRMS[i] = a * relacaoTC();    // primário (RTC)
    }
    else {
      correnteSec[i] = 0;
      correnteRMS[i] = 0;
    }
  }

  canalMedicao++;

  if (canalMedicao >= 6) {
    canalMedicao = 0;
    rodadaCompleta = true;
  }
}

bool tensoesCalibradas() {
  for (int i = 0; i < 3; i++) {
    if (!(calibTensao[i].zeroCalibrado && calibTensao[i].kCalculado)) return false;
  }
  return true;
}

bool correntesCalibradas() {
  for (int i = 0; i < 3; i++) {
    if (!(calibCorrente[i].zeroCalibrado && calibCorrente[i].kCalculado)) return false;
  }
  return true;
}

// ============================================================
// FUNÇÕES DE PROTEÇÃO
// ============================================================
// Tempo da curva IEC (Eq. 03 e Eq. 04) - válido para M > 1
float calcularTempo51(float m) {

  float t;

  if (cfg.curva51 == CURVA_SI) {
    // t = 0,14 / (M^0,02 - 1) * TMS
    float den = powf(m, 0.02f) - 1.0f;
    if (den <= 0.0f) return 1.0e9f;
    t = (0.14f / den) * cfg.tms51;
  }
  else {
    // t = 80 / (M^2 - 1) * TMS
    float den = (m * m) - 1.0f;
    if (den <= 0.0f) return 1.0e9f;
    t = (80.0f / den) * cfg.tms51;
  }

  return t;
}

void resetarTemporizadores() {
  temporizando51 = false;
  confirmando27 = false;
  confirmando59 = false;
}

void executarProtecoes() {

  // Falha já atuada: aguarda RESET
  if (falhaAtiva) {
    resetarTemporizadores();
    return;
  }

  // Só protege com o disjuntor fechado
  if (!disjuntorFechado) {
    resetarTemporizadores();
    return;
  }

  // ==========================================================
  // PROTEÇÕES DE CORRENTE (ANSI 50 e 51)
  // Maior corrente de fase, em amperes primários
  // ==========================================================
  if (correntesCalibradas()) {

    float iMax = correnteRMS[0];
    if (correnteRMS[1] > iMax) iMax = correnteRMS[1];
    if (correnteRMS[2] > iMax) iMax = correnteRMS[2];

    // ---------------- ANSI 50 (Eq. 01) ----------------------
    // Imedida >= Ipickup  =>  Trip
    if (iMax >= cfg.pickup50) {
      dispararProtecao(FUNC_ANSI50);
      return;
    }

    // ---------------- ANSI 51 (Eq. 02, 03, 04) --------------
    // M = Imedida / Ipickup ; se M > 1 inicia temporização
    float m = iMax / cfg.pickup51;

    if (m > 1.0f) {

      float t = calcularTempo51(m);

      if (!temporizando51) {
        temporizando51 = true;
        inicio51 = millis();
      }

      // tempo decorrido >= t  =>  Trip
      if ((float)(millis() - inicio51) >= t * 1000.0f) {
        dispararProtecao(FUNC_ANSI51);
        return;
      }
    }
    else {
      temporizando51 = false;
    }
  }
  else {
    temporizando51 = false;
  }

  // ==========================================================
  // PROTEÇÕES DE TENSÃO (ANSI 27 e 59)
  // Vnominal = 115 V x RTP (tensão primária nominal)
  // ==========================================================
  if (tensoesCalibradas()) {

    float vMin = tensaoRMS[0];
    float vMax = tensaoRMS[0];

    for (int i = 1; i < 3; i++) {
      if (tensaoRMS[i] < vMin) vMin = tensaoRMS[i];
      if (tensaoRMS[i] > vMax) vMax = tensaoRMS[i];
    }

    float vNominal = TP_SECUNDARIO * relacaoTP();

    // ---------------- ANSI 27 (Eq. 05 e 06) -----------------
    float vLimite27 = (cfg.pct27 * vNominal) / 100.0f;

    if (vMin < vLimite27) {

      if (!confirmando27) {
        confirmando27 = true;
        inicio27 = millis();
      }

      if (millis() - inicio27 >= CONFIRMACAO_TENSAO_MS) {
        dispararProtecao(FUNC_ANSI27);
        return;
      }
    }
    else {
      confirmando27 = false;
    }

    // ---------------- ANSI 59 (Eq. 07 e 08) -----------------
    float vLimite59 = (cfg.pct59 * vNominal) / 100.0f;

    if (vMax > vLimite59) {

      if (!confirmando59) {
        confirmando59 = true;
        inicio59 = millis();
      }

      if (millis() - inicio59 >= CONFIRMACAO_TENSAO_MS) {
        dispararProtecao(FUNC_ANSI59);
        return;
      }
    }
    else {
      confirmando59 = false;
    }
  }
  else {
    confirmando27 = false;
    confirmando59 = false;
  }
}

// ============================================================
// ATUAÇÃO DA PROTEÇÃO
// Falha detectada: identifica função, captura grandezas,
// abre disjuntor, registra evento e atualiza a tela.
// O trip é acionado ANTES da gravação em EEPROM (mais lenta).
// ============================================================
void dispararProtecao(uint8_t funcao) {

  Evento e = montarEvento(funcao);      // data/hora, V, I, estado

  falhaAtiva = true;
  funcaoAtuada = funcao;

  acionarTrip();                        // abre disjuntor
  resetarTemporizadores();

  gravarEvento(e);                      // registra evento (EEPROM)

  Serial.println("================================");
  Serial.print("TRIP: ");
  Serial.println(nomeFuncao(funcao));
  Serial.println("================================");

  // Atualiza tela
  if (telaAtual == SUPERVISAO) {
    atualizarTelaSupervisao();
  }
  else if (telaAtual == MENU_PRINCIPAL) {
    atualizarRodape();
  }
}

// ============================================================
// EEPROM
// ============================================================
void inicializarCalibracaoPadrao() {

  for (int i = 0; i < 3; i++) {
    calibTensao[i].zeroRMS = 0;
    calibTensao[i].referenciaRMS = 0;
    calibTensao[i].valorReferencia = TENSAO_REFERENCIA;
    calibTensao[i].fatorK = 0;
    calibTensao[i].zeroCalibrado = false;
    calibTensao[i].referenciaCalibrada = false;
    calibTensao[i].kCalculado = false;

    calibCorrente[i].zeroRMS = 0;
    calibCorrente[i].referenciaRMS = 0;
    calibCorrente[i].valorReferencia = CORRENTE_REFERENCIA;
    calibCorrente[i].fatorK = 0;
    calibCorrente[i].zeroCalibrado = false;
    calibCorrente[i].referenciaCalibrada = false;
    calibCorrente[i].kCalculado = false;
  }
}

void inicializarConfigPadrao() {
  cfg.tcPrimario = 5.0;        // RTC = 1
  cfg.tpPrimario = 115.0;      // RTP = 1
  cfg.pickup50 = 20.0;
  cfg.pickup51 = 6.0;
  cfg.tms51 = 0.5;
  cfg.curva51 = CURVA_SI;
  cfg.pct27 = 80.0;
  cfg.pct59 = 110.0;
}

void inicializarHistoricoPadrao() {
  memset(&hist, 0, sizeof(hist));
  hist.magic = EEPROM_MAGIC_HIST;
}

void salvarCalibracaoEEPROM() {

  DadosCalib dados;
  dados.magic = EEPROM_MAGIC_CALIB;

  for (int i = 0; i < 3; i++) {
    dados.tensao[i] = calibTensao[i];
    dados.corrente[i] = calibCorrente[i];
  }

  EEPROM.put(EEPROM_END_CALIB, dados);

  Serial.println("CALIBRACAO SALVA NA EEPROM");
}

void carregarCalibracaoEEPROM() {

  DadosCalib dados;
  EEPROM.get(EEPROM_END_CALIB, dados);

  if (dados.magic != EEPROM_MAGIC_CALIB) {
    Serial.println("EEPROM sem calibracao valida.");
    return;
  }

  for (int i = 0; i < 3; i++) {
    calibTensao[i] = dados.tensao[i];
    calibCorrente[i] = dados.corrente[i];
  }

  Serial.println("CALIBRACAO CARREGADA DA EEPROM");
}

void salvarConfigEEPROM() {

  DadosConfig dados;
  dados.magic = EEPROM_MAGIC_CONFIG;
  dados.cfg = cfg;

  EEPROM.put(EEPROM_END_CONFIG, dados);

  Serial.println("AJUSTES SALVOS NA EEPROM");
}

void carregarConfigEEPROM() {

  DadosConfig dados;
  EEPROM.get(EEPROM_END_CONFIG, dados);

  if (dados.magic != EEPROM_MAGIC_CONFIG) {
    Serial.println("EEPROM sem ajustes validos. Usando padrao.");
    salvarConfigEEPROM();
    return;
  }

  cfg = dados.cfg;

  // Proteção contra dados corrompidos
  if (cfg.curva51 > CURVA_EI) cfg.curva51 = CURVA_SI;

  Serial.println("AJUSTES CARREGADOS DA EEPROM");
}

void salvarHistoricoEEPROM() {
  EEPROM.put(EEPROM_END_HIST, hist);
}

void carregarHistoricoEEPROM() {

  DadosHist dados;
  EEPROM.get(EEPROM_END_HIST, dados);

  if (dados.magic != EEPROM_MAGIC_HIST ||
      dados.proximo >= MAX_EVENTOS ||
      dados.total > MAX_EVENTOS) {

    Serial.println("EEPROM sem historico valido.");
    inicializarHistoricoPadrao();
    salvarHistoricoEEPROM();
    return;
  }

  hist = dados;

  Serial.println("HISTORICO CARREGADO DA EEPROM");
}
