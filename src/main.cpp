#include <mbed.h>
#include <vector>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h" 
#include "drivers/TS_DISCO_F429ZI.h"

//SPI pins: mosi, miso, sclk, cs
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); 
//Interrupt pin definition
InterruptIn int2(PA_2,PullDown);

#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts                 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

//define the timer to record the running time
Timer timer;
// define the flags to be used for the interrupt and spi communication
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define TS_FLAG_Record 3
#define TS_FLAG_Attempt 4
EventFlags flags;
EventFlags TS_flag; // this is for interrupting the touch screen

// initialize the built-in LCD display and touch screen
LCD_DISCO_F429ZI lcd; // Initialize the built-in LCD display
TS_DISCO_F429ZI ts; // Initialize the built-in touch screen

// Button 1
#define BUTTON1_X_START 30
#define BUTTON1_X_END   220
#define BUTTON1_Y_START 145
#define BUTTON1_Y_END   195

// Button 2
#define BUTTON2_X_START 30
#define BUTTON2_X_END   220
#define BUTTON2_Y_START 225
#define BUTTON2_Y_END   275


// variables for Gaussian filter
#define sigma 1.0
#define K  1

// define the state of the button pressed
bool button1_pressed = false;
bool button2_pressed = false;
// define the state of the button pressed
bool record_pressed = false;
bool attempt_pressed = false;

// define the write and read buffers for the spi communication
uint8_t write_buf[32];
uint8_t read_buf[32];

// define the raw and actual values of the gyroscope
int16_t raw_gx,raw_gy,raw_gz = 0;
// define the actual values of the gyroscope
float gx, gy, gz = 0;

// we will store the maximum sequence of 1000 samples.
const int MAX_SEQUENCE = 100;
// define the float array to store the recorded sequence
float recorded_sequence[MAX_SEQUENCE][3];
int recorded_sequence_raw[MAX_SEQUENCE][3];
// define the float array to store the attempted sequence
float attempted_sequence[MAX_SEQUENCE][3];
int attempted_sequence_raw[MAX_SEQUENCE][3];
// define the vector to store the several recorded sequences
// currently, we allowed 5 sequences to be recorded for future checking
const int recorded_size = 5;
const int attempted_size = 3;
std::vector<std::vector<float>> recorded_sequences;
std::vector<std::vector<int>> recorded_sequences_raw;
// define the vector to store the several attempted sequences to future verifying
std::vector<std::vector<float>> attempted_sequences;
std::vector<std::vector<int>> attempted_sequences_raw;

//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

void data_cb(){
  flags.set(DATA_READY_FLAG);
};

void button1_handler() {
  TS_flag.set(TS_FLAG_Record);
  button1_pressed = true;
}

void button2_handler() {
  TS_flag.set(TS_FLAG_Attempt);
  button2_pressed = true;
}

void detect_touch_events(){
  // detect the touch screen events
  TS_StateTypeDef TS_State;
  ts.GetState(&TS_State);

  if (TS_State.TouchDetected) {
    uint16_t x = TS_State.X; 
    uint16_t y = 320 - TS_State.Y;
    // if the touch screen is pressed, we will record the sequence
    if (x >= BUTTON1_X_START && x <= BUTTON1_X_END && y >= BUTTON1_Y_START && y <= BUTTON1_Y_END) {
      button1_handler();
    }

    // Check if the touch event occurred within the button2 area
    if (x >= BUTTON2_X_START && x <= BUTTON2_X_END && y >= BUTTON2_Y_START && y <= BUTTON2_Y_END) {
      button2_handler();
    }
  
  }
}
// intialize the gyroscope
void init_gyroscope(){
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,3);
  spi.frequency(1'000'000);

  write_buf[0]=CTRL_REG1;
  write_buf[1]=CTRL_REG1_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG4;
  write_buf[1]=CTRL_REG4_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG3;
  write_buf[1]=CTRL_REG3_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

}

// gryoscope read function 
void read_gryscope(){
  //prepare the write buffer to trigger a sequential read
  write_buf[0]=OUT_X_L|0x80|0x40;

  //start sequential sample reading
  spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
  //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
  raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
  raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
  raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

  // printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);
  // 17.5f*0.017453292519943295769236907684886f / 1000.0f = 0.000305432619f
  gx=raw_gx*0.000305432619f;
  gy=raw_gy*0.000305432619f;
  gz=raw_gz*0.000305432619f;

  // printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);
}

// record the sequence of the gyroscope data x,y and z
void record_sequence(){
  DigitalOut led1(LED1);
  for(int i=0;i<MAX_SEQUENCE;i++){
    printf("Recording sequence %d\n",i);
    read_gryscope();
    recorded_sequence[i][0] = gx;
    recorded_sequence[i][1] = gy;
    recorded_sequence[i][2] = gz;
    recorded_sequence_raw[i][0] = raw_gx;
    recorded_sequence_raw[i][1] = raw_gy;
    recorded_sequence_raw[i][2] = raw_gz;
    led1 =1;
    // printf("Recorded|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",recorded_sequence[i][0],recorded_sequence[i][1],recorded_sequence[i][2]);
    ThisThread::sleep_for(100ms);
    //led1 =1;
    // thread_sleep_for(500);
    led1= 0;
    // thread_sleep_for(500);
    recorded_sequences.push_back({recorded_sequence[i][0],recorded_sequence[i][1],recorded_sequence[i][2]});
  }
  // store the recorded sequence into the vector
  // record_sequences push back the recorded sequence
  // recorded_sequences.push_back({recorded_sequence})
  // recorded_sequences.push_back(std::vector<float>(recorded_sequence[0],recorded_sequence[0]+3*MAX_SEQUENCE));
//   recorded_sequences_raw.push_back(std::vector<int>(recorded_sequence_raw[0],recorded_sequence_raw[0]+3*MAX_SEQUENCE));
}
// attempt the sequence of the gyroscope data x,y and z
void attempt_sequence(){
  DigitalOut led1(LED1);
  for(int i=0;i<MAX_SEQUENCE;i++){
    printf("Attempting sequence %d\n",i);
    read_gryscope();
    attempted_sequence[i][0] = gx;
    attempted_sequence[i][1] = gy;
    attempted_sequence[i][2] = gz;
    attempted_sequence_raw[i][0] = raw_gx;
    attempted_sequence_raw[i][1] = raw_gy;
    attempted_sequence_raw[i][2] = raw_gz;
    led1 =1;
    // printf("Attempted|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",attempted_sequence[i][0],attempted_sequence[i][1],attempted_sequence[i][2]);
    ThisThread::sleep_for(100ms);
    led1 =0;
    attempted_sequences.push_back({attempted_sequence[i][0],attempted_sequence[i][1],attempted_sequence[i][2]});
  }
  // store the attempted sequence into the vector
  // attempted_sequences.push_back(std::vector<float>(attempted_sequence[0],attempted_sequence[0]+3*MAX_SEQUENCE));
}
// lcd display initialization
void setup_background_layer(){
  #define BACKGROUND 1
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

// normalize the recorded or attempted sequence
void normalize_sequence(std::vector<std::vector<float>> &sequence){
  if (sequence.empty()) {
    return;
  }

  int num_points = sequence.size();
  int num_dimensions = sequence[0].size();

  std::vector<float> mean(num_dimensions, 0.0f);
  std::vector<float> stddev(num_dimensions, 0.0f);

  // Calculate mean
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < num_dimensions; ++j) {
      mean[j] += sequence[i][j];
    }
  }

  for (int j = 0; j < num_dimensions; ++j) {
    mean[j] /= num_points;
  }

  // Calculate standard deviation
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < num_dimensions; ++j) {
      stddev[j] += (sequence[i][j] - mean[j]) * (sequence[i][j] - mean[j]);
    }
  }

  for (int j = 0; j < num_dimensions; ++j) {
    stddev[j] = std::sqrt(stddev[j] / num_points);
  }

  // Normalize sequence
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < num_dimensions; ++j) {
      if (stddev[j] != 0) {
        sequence[i][j] = (sequence[i][j] - mean[j]) / stddev[j];
      }
    }
  }
}

// apply moving average filter to the recorded or attempted sequence
void average_filter(std::vector<std::vector<float>> &sequence, int window_size = 5){
  if (sequence.empty() || window_size <= 1) {
    return;
  }

  int num_points = sequence.size();
  int num_dimensions = sequence[0].size();

  std::vector<std::vector<float>> filtered_sequence(num_points, std::vector<float>(num_dimensions, 0.0f));
  
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < num_dimensions; ++j) {
      int window_start = std::max(0, i - window_size / 2);
      int window_end = std::min(num_points - 1, i + window_size / 2);

      float sum = 0;
      for (int k = window_start; k <= window_end; ++k) {
        sum += sequence[k][j];
      }

      filtered_sequence[i][j] = sum / (window_end - window_start + 1);
    }
  }
  sequence = filtered_sequence;

}

// try to implement Gaussian Filter here

void gaussian_filter(std::vector<std::vector<float>> &sequence, int window_size = 5){
  if (sequence.empty() || window_size <= 1) {
    return;
  }

  int num_points = sequence.size();
  int num_dimensions = sequence[0].size();

  std::vector<std::vector<float>> filtered_sequence(num_points, std::vector<float>(num_dimensions, 0.0f));
  
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < num_dimensions; ++j) {
      int window_start = i - (num_points - 1) / 2.0;
      int window_end = j - (num_points - 1) / 2.0;
      sequence[i][j] = K * exp(((pow(window_start, 2) + pow(window_end, 2)) / ((2 * pow(sigma, 2)))) * (-1));

      float sum = 0;
      for (int k = window_start; k <= window_end; ++k) {
        sum += sequence[k][j];
      }

      filtered_sequence[i][j] = sum / (window_end - window_start + 1);
    }
  }
  sequence = filtered_sequence;

}


// compare the recorded sequence and attempted sequence using DTW algorithm
float compare_sequence(std::vector<std::vector<float>> &seq1, std::vector<std::vector<float>> &seq2){
  // use the DTW algorithm to compare the two sequences
  // normalize the two sequences
  // normalize_sequence(seq1);
  // normalize_sequence(seq2);
  // apply the moving average filter to the two sequences
  average_filter(seq1, 5);
  average_filter(seq2, 5);
  // get the size of the two sequences
  int m = seq1.size();
  int n = seq2.size();

  std::vector<std::vector<float>> dtw(m + 1, std::vector<float>(n + 1, std::numeric_limits<float>::infinity()));
  dtw[0][0] = 0;

  for (int i = 1; i <= m; ++i) {
    for (int j = 1; j <= n; ++j) {
      float cost = 0;
      for (int k = 0; k < 3; ++k)
      {
        cost += std::abs(seq1[i - 1][k] - seq2[j - 1][k]);
      }
      dtw[i][j] = cost + std::min({dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1]});
    }
  }
  printf("dtw[m][n]: %f\n",dtw[m][n]);
  return dtw[m][n];
  

}
void compare_try(){
  ThisThread::sleep_for(1000ms);
  printf("start comparing\n");
}
int main() {
  //intialize the gyroscope
  init_gyroscope();
  // configure the interrupt to call our function
  // when the pin becomes high
  int2.rise(&data_cb);
  // manually check the signal and set the flag
  if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
    flags.set(DATA_READY_FLAG);
  }
  flags.wait_all(DATA_READY_FLAG);
  
  // initialize the lcd
  setup_background_layer();
  // Display the title on the screen
  lcd.DisplayStringAt(8, 30, (uint8_t *)"2023 Gesture Unlock", CENTER_MODE);
  lcd.DisplayStringAt(10, 50, (uint8_t *)"RTES Project", CENTER_MODE);
  lcd.DisplayStringAt(10, 70, (uint8_t *)"Team 40", CENTER_MODE);
  lcd.DisplayStringAt(10, 90, (uint8_t *)"zw3464   yl10673", CENTER_MODE);
  // Display the buttons on the screen
  lcd.DisplayStringAt(10, 160, (uint8_t *)"Record Button", CENTER_MODE);
  lcd.DisplayStringAt(10, 240, (uint8_t *)"Attempt Button", CENTER_MODE);

  // Display the button rectuangles on the screen
  lcd.DrawRect(30, 145, 190, 50);
  lcd.DrawRect(30, 225, 190, 50);

  while (1) {
    // detect the touch events
    detect_touch_events();
    // if the button1 is pressed, record the sequence
    if (button1_pressed) {
      lcd.DisplayStringAt(10, 160, (uint8_t *)" Recording... ", CENTER_MODE);
      button1_pressed = false;
      record_sequence();
      // print the recorded sequence
      for(int i=0;i<MAX_SEQUENCE;i++){
        printf("Recorded|\tgx: %d \t gy: %d \t gz: %d\n",recorded_sequence_raw[i][0],recorded_sequence_raw[i][1],recorded_sequence_raw[i][2]);
        // printf("Recorded|\tgx: %f \t gy: %f \t gz: %f\n",recorded_sequence[i][0],recorded_sequence[i][1],recorded_sequence[i][2]);
      }
      printf("Recording!");
      lcd.DisplayStringAt(10, 160, (uint8_t *)"Record Button", CENTER_MODE);
      // recorded the button pressed
      record_pressed = true;
    }
    
    // if the button2 is pressed, attempt to unlock
    if (button2_pressed) {
      lcd.DisplayStringAt(10, 240, (uint8_t *)" Attempting... ", CENTER_MODE);
      button2_pressed = false;
      attempt_sequence();
      // print the attemped sequence
      for(int i=0;i<MAX_SEQUENCE;i++){
        printf("Recorded|\tgx: %d \t gy: %d \t gz: %d\n",attempted_sequence_raw[i][0],attempted_sequence_raw[i][1],attempted_sequence_raw[i][2]);
        // printf("Recorded|\tgx: %f \t gy: %f \t gz: %f\n",recorded_sequence[i][0],recorded_sequence[i][1],recorded_sequence[i][2]);
      }
      printf("Attempting!");
      lcd.DisplayStringAt(10, 240, (uint8_t *)"Attempt Button", CENTER_MODE);
      // attempt the button pressed
      attempt_pressed = true;
    }

    if(attempt_pressed && record_pressed){
      attempt_pressed = false;
      record_pressed = false;
      printf("start comparing\n");
      // compare the recorded sequence and attempted sequence
      float distance = compare_sequence(recorded_sequences,attempted_sequences);
      printf("distance: %f\n",distance);
      
      // printf("distance*100 : %d\n",distance*100);
      if(distance<5000){
        lcd.DisplayStringAt(10, 240, (uint8_t *)"Unlock Success!", CENTER_MODE);
      }else{
        lcd.DisplayStringAt(10, 240, (uint8_t *)"Unlock Failed!", CENTER_MODE);
      }
      // compare_try();
      
    }

  }
}
