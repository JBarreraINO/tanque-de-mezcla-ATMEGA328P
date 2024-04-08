#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>

#define set_bit(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define clear_bit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// Definición de pines y puertos
#define SENSOR_PROXIMITY_A_PIN PD2  // Sensor de proximidad A
#define SENSOR_PROXIMITY_B_PIN PD3  // Sensor de proximidad B
#define START_BUTTON_PIN PD4        // Pulsador de inicio
#define EMERGENCY_STOP_PIN PB4      // Pulsador de parada de emergencia
#define PWM_MOTOR_PIN PD6           // Pin para el motor PWM
#define START_LED_PIN PB0             // LED indicador de mezcla
#define STOP_LED_PIN PC0            // LED indicador de STOP
#define EMPTYING_LED_PIN PB2        // LED indicador de STOP
#define SERVO_PIN PD5               // Pin para el servo motor
#define DIGITAL_OUTPUT1_PIN DDC2    // Salida digital 1
#define DIGITAL_OUTPUT2_PIN DDC3    // Salida digital 2
#define BUZZER_PIN PC4              // Buzzer
#define LED_PIN PB5                 // Define el pin al que está conectado el LED


//SERVO

#define BUZZ_COUNT_MAX 2  // Número máximo de veces que se debe encender/apagar el zumbador
// Estados del proceso
#define IDLE_STATE 0
#define MIXING_STATE 1
#define EMPTYING_STATE 2
#define FINAL_STATE 3

volatile uint8_t IngredienteA = 0;
volatile uint8_t IngredienteB = 0;
// Variables globales
volatile uint8_t state = IDLE_STATE;
volatile uint8_t proximity_A_count;
volatile uint8_t proximity_B_count;
volatile uint8_t flagFinal=0;
volatile uint16_t seconds = 0;
volatile uint16_t mixing = 0;
volatile uint8_t BUZZER_STATE = 0;
volatile uint8_t BUZZER_count = 0;

volatile uint8_t buzz_count = 0;             // Contador para el número de veces que el zumbador se ha encendido/apagado
volatile uint8_t digital_output_active = 0;  // Variable para rastrear si las señales de control digital están activas
volatile uint8_t estado_pulsador_asc = 0;    // estado actual del pulsador
volatile uint8_t lastButtonState_asc = 0;    // estado anterior del pulsador
volatile uint8_t estado_pulsador_ascB = 0;   // estado actual del pulsador
volatile uint8_t lastButtonState_ascB = 0;   // estado anterior del pulsador

volatile uint8_t estado_pulsador_ascE = 0;   // estado actual del pulsador
volatile uint8_t lastButtonState_ascE = 0;   // estado anterior del pulsador





// Función para inicializar periféricos
void init_peripherals() {
  // Configurar el pin del sensor de proximidad A como entrada
  DDRD &= ~(1 << SENSOR_PROXIMITY_A_PIN);
  // Configurar el pin del sensor de proximidad B como entrada
  DDRD &= ~(1 << SENSOR_PROXIMITY_B_PIN);
  // Configurar el pin del botón de inicio como entrada
  DDRD &= ~(1 << START_BUTTON_PIN);
  // Configurar el pin del botón de parada de emergencia como entrada
  DDRB &= ~(1 << EMERGENCY_STOP_PIN);

  //SALIDAS
  DDRD |= (1 << PWM_MOTOR_PIN);  // PWM motor como salida
  DDRB |= (1 << START_LED_PIN);    // LED de mezcla como salida

  // Configurar el pin del LED de STOP como salida
  DDRC |= (1 << STOP_LED_PIN);
  DDRB |= (1 << LED_PIN);
  DDRB |= (1 << PB1);  // Configura el pin del LED como salida
  DDRB |= (1 << PB2);  // Configura el pin del LED como salida
                       // Configurar el pin del servo como salida

  DDRD |= (1 << SERVO_PIN);





  // Configurar el pin de la salida digital 1 como salida
  DDRC |= (1 << DIGITAL_OUTPUT1_PIN);

  // Configurar el pin de la salida digital 2 como salida
  DDRC |= (1 << DIGITAL_OUTPUT2_PIN);

  // Configurar el pin del buzzer como salida
  DDRC |= (1 << BUZZER_PIN);


  // Configurar pull- DOWN PARA LA ENTRADA
  //PORTD &= ~(1 << SENSOR_PROXIMITY_A_PIN) | (1 << SENSOR_PROXIMITY_B_PIN) | (1 << START_BUTTON_PIN) | (1 << EMERGENCY_STOP_PIN);

  // Configurar temporizador para PWM

  TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);  // Modo PWM rápido, no inversor
  TCCR0B |= (1 << CS00);                                  // Sin preescalador
                                                          // Inicializa el PWM en 0

  // Configurar el modo PWM (Fast PWM) en el Timer/Counter0
  TCCR0A |= (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS01);  // Prescaler de 8 (para una frecuencia de 1 MHz)



  //para el servo
  TCCR0B &= ~(1 << WGM02);
  TCCR0A |= (1 << WGM01);
  TCCR0A |= (1 << WGM00);
  OCR0A = 0;
  // Prescalador 1024
  TCCR0B |= (1 << CS02);
  TCCR0B &= ~(1 << CS01);
  TCCR0B |= (1 << CS00);

  set_bit(TCCR0A, COM0B1);
  clear_bit(TCCR0A, COM0B0);


  cli();

  // Configurar Timer1 para temporización
  TCCR1B |= (1 << WGM12);               // Modo CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Preescalador de 1024
  OCR1A = 7813;                         // Valor de comparación para temporizador de 1 segundo "se usa oscilador de 16mhz con preescalar 1024"
  TIMSK1 |= (1 << OCIE1A);
  // calculo
  /*
CALCULO PARA 1S
16MHZ/1024 =15625hz
1/15625= 0.000064 segundos teniendo el periodo se puede desarrollar para cualquier tiempo deseado 
ejemplo para: 0.5s/0.000064=7813
 */

  sei();
  // Habilitar interrupción de comparación A

 // EIMSK |= (1 << INT0);  // Habilita la interrupción externa INT0
}

// Función para activar el motor PWM con un cierto ciclo de trabajo
void set_pwm_duty_cycle(uint8_t duty_cycle) {
  OCR0A = duty_cycle;
}


// Función para activar el servo en una posición específica (0 a 255)
void set_servo_position(uint16_t position) {
  // Convertir el valor de posición a un valor válido dentro del rango del servo
  OCR0B = position;
}




// Función para manejar la lógica de control del proceso
void process_control_logic() {

  if (state == IDLE_STATE) {
     
    if ((PIND & (1 << START_BUTTON_PIN)) || flagFinal== 1) {
      PORTC &= ~(1 << BUZZER_PIN);  // Apagar el zumbador
      sei();
      IngredienteA = 0;
      IngredienteB = 0;
      proximity_B_count = 0;
      proximity_A_count = 0;
      mixing = 0;
      seconds = 0;
      flagFinal=0;
      PORTB |= (1 << START_LED_PIN);  // Encender LED de mezcla
      set_pwm_duty_cycle(102);      // 40% de ciclo útil para el motor
      set_servo_position(0);        // Abrir la compuerta
      state = MIXING_STATE;
      PORTC |= (1 << PORTC2);  // Activar salidas digitales
      PORTC |= (1 << PORTC3);  // Activar salidas digitales
    }

  }

  else if (state == MIXING_STATE) {

    if ((PIND & (1 << SENSOR_PROXIMITY_A_PIN))) {

      PORTB |= (1 << LED_PIN);
      lastButtonState_asc = 1;
    }


    if (!(PIND & (1 << SENSOR_PROXIMITY_A_PIN))) {

      PORTB &= ~(1 << LED_PIN);
      estado_pulsador_asc = 1;
    }


    if ((lastButtonState_asc == 1 && estado_pulsador_asc == 1)) {
      proximity_A_count++;
      lastButtonState_asc = 0;
      estado_pulsador_asc = 0;
    }


    if ((PIND & (1 << SENSOR_PROXIMITY_B_PIN))) {

      PORTB |= (1 << LED_PIN);
      lastButtonState_ascB = 1;
    }


    if (!(PIND & (1 << SENSOR_PROXIMITY_B_PIN))) {
      PORTB &= ~(1 << LED_PIN);
      estado_pulsador_ascB = 1;
    }


    if ((lastButtonState_ascB == 1 && estado_pulsador_ascB == 1)) {
      proximity_B_count++;
      lastButtonState_ascB = 0;
      estado_pulsador_ascB = 0;
    }


    // Verificar si se completaron las cantidades de A y B SE PONE EL DOBLE  DEL VALOR PARA CONTAR FLACO DE SUBIDA Y BAJADA
    if (proximity_A_count >= 30) {
      // Desactivar señales de control digital
      PORTC &= ~(1 << PORTC3);
      IngredienteA = 1;
    }

    if (proximity_B_count >= 10) {  //SE PONE EL DOBLE  DEL VALOR PARA CONTAR FLACO DE SUBIDA Y BAJADA
      // Desactivar señales de control digital
      PORTC &= ~(1 << PORTC2);
      IngredienteB = 1;
    }


    if (IngredienteA == 1 && IngredienteB == 1) {
      // Aumentar velocidad de agitación
      set_pwm_duty_cycle(204);  // 80% de ciclo útil para el motor
      PORTB |= (1 << PORTB1);   //enciende piloto de mezclado
      state = EMPTYING_STATE;
    }

  }


  else if (state == EMPTYING_STATE) {
  }
      if ((PINB & (1 << EMERGENCY_STOP_PIN))) {
        // Detener el proceso de mezcla
       // state = IDLE_STATE;
        set_pwm_duty_cycle(0); // Detener el motor
        // Conservar la posición del servo motor (si es necesario)
        // Activar salida digital de alarma (buzzer)
        PORTC |= (1 << BUZZER_PIN); // Encender el buzzer
        PORTC &= ~(1 << PORTC2);
          PORTC &= ~(1 << PORTC3);
          PORTB &= ~(1 << START_LED_PIN);
          PORTB &= ~(1 << LED_PIN);
          PORTB &= ~(1 << PB1);
          PORTB &= ~(1 << PB2);
         
          //cli();  
    }

    if ((PINB & (1 << EMERGENCY_STOP_PIN))) {

      PORTB |= (1 << LED_PIN);
      lastButtonState_ascE = 1;
    }


    if (!(PIND & (1 << EMERGENCY_STOP_PIN))) {
      PORTB &= ~(1 << LED_PIN);
      estado_pulsador_ascE = 1;
    }


    if ((lastButtonState_ascE == 1 && estado_pulsador_ascE == 1)) {
    
      lastButtonState_ascE = 0;
      estado_pulsador_ascE = 0;
      state =IDLE_STATE;
    }


}



ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;

  if (state == EMPTYING_STATE) {
    mixing++;
    // Incrementar contador de segundos
    // Verificar si ha transcurrido 1 minuto
    // las unidades de comparacion equivale a 0.5s por lo tanto para 60s la unidad es 120
    // Detener el motor y vaciar el tanque
    if (mixing > 120) {
      mixing = 0;
      set_pwm_duty_cycle(0);   // Detener el motor
      set_servo_position(26);  // Abrir la compuerta
      PORTC |= (1 << PORTC0);  // Encender LED de STOP
      PORTB &= ~(1 << PORTB1);
      state = FINAL_STATE;
    }

  } else if (state == FINAL_STATE) {

    seconds++;
    PORTB |= (1 << PORTB2);  //enciende piloto de mezclado
    if (seconds > 90) {
      PORTB &= ~(1 << PORTB2);
      if (buzz_count < BUZZ_COUNT_MAX * 2) {  // Si aún no se ha alcanzado el número máximo de encendidos/apagados
        BUZZER_STATE = !BUZZER_STATE;         // Invertir el estado del zumbador

        // Escribir el nuevo estado del zumbador directamente en el registro de puerto
        if (BUZZER_STATE) {
          PORTC |= (1 << BUZZER_PIN);  // Encender el zumbador
        } else {
          PORTC &= ~(1 << BUZZER_PIN);  // Apagar el zumbador
        }
        buzz_count++;  // Incrementar el contador

      }

      else {
        flagFinal=1;
        buzz_count = 0;
        state = IDLE_STATE;
      }
    }
  }
}

int main() {
  // Inicializar periféricos
  init_peripherals();

  // Habilitar interrupciones globales


  // Lógica de control del proceso
  while (1) { process_control_logic(); }



  return 0;
}