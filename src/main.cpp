#include <mbed.h>
#include <vector>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h" 
#include "drivers/TS_DISCO_F429ZI.h"
#include <thread.h>

/*
Define the required SPI, Interrupt, LED and Thread objects
*/
// Define the SPI communication pins with MOSI, MISO, SCLK, and CS
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); 
// Define the Interrupt pin for the gyroscope
InterruptIn int2(PA_2,PullDown);
// Define the InterruptIn pin for the user button
InterruptIn button(USER_BUTTON);
// Define the Thread for the LCD display
Thread lcd_thread;
// Define the Led for the LCD display indicating recording or attempting
DigitalOut led1(LED2);

/*
Define the gyroscope registers based on the demo code and datasheet
*/
//address of first register with gyro data
#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0x6F
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,245 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0x00
//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts                 
#define CTRL_REG3_CONFIG 0x08

/*
Define the event flags for the gyroscope and touch screen
*/
#define SPI_FLAG (1 << 0)             // SPI transfer complete
#define DATA_READY_FLAG (1 << 1)      // Data ready from gyroscope
#define TS_FLAG_Record (1 << 2)       // Record button pressed on the touch screen
#define TS_FLAG_Attempt (1 << 3)      // Attempt button pressed on the touch screen
#define Record_Flag (1 << 4)          // Recording sequence
#define Attempt_Flag (1 << 5)         // Attempting sequence
#define Compare_Flag (1 << 6)         // Comparing sequence between recorded and attempted
#define Compare_Done_Flag (1 << 7)    // Compare finished 
#define Locked_Flag (1 << 8)          // Gesture still locked
#define Unlocked_Flag (1 << 9)        // Gesture unlocked successfully
#define Attempt_Limit_Flag (1 << 10)  // Attempt limit reached

// define the flags which will be used for the main and lcd thread
EventFlags flags;          // Event flags for main thread
EventFlags button_event;   // Event flags for LCD thread  

// initialize the built-in LCD display and touch screen
LCD_DISCO_F429ZI lcd;      // Initialize the built-in LCD display
TS_DISCO_F429ZI ts;        // Initialize the built-in touch screen
TS_StateTypeDef TS_State;  // Touch screen state

/*
Define the button area for the Record and Attempt button
*/
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

// define the state of the button pressed
bool button1_pressed = false;
bool button2_pressed = false;

// define the state of clear the log information on the screen 
bool clear_flag = false;  // used for the user button

/*
Define the temp variables for the gyroscope processed data and raw data
Also, including the vector to store the recorded and attempted sequence
Also, including the attempting times, max sequence and similiarity distance
*/
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
// define the float array to store the attempted sequence
float attempted_sequence[MAX_SEQUENCE][3];
// define the vector to store the several recorded sequences
std::vector<std::vector<float>> recorded_sequences;
// define the vector to store the several attempted sequences
std::vector<std::vector<float>> attempted_sequences;

// define the similiarity distance as infinty
float similiarity = std::numeric_limits<float>::infinity();
float similiarity_threshold = 110.0f;
// we allow the attmepted sequence to be 4 times
const int attempted_size = 3;
// temp variable to store the attempt times
int attempt_time = 0;

/*
all the function we have, we declare here first to show the structure of the code
*/
// define the function to handle the spi communication and data ready callback
void spi_cb(int event);
void data_cb();

// define the function to handle the button pressed 
void button1_handler(); // this is for the record button
void button2_handler(); // this is for the attempt button
void button2_handler2(); // this is for the attempt button after the failure attempts
void outside_button_handler(); // this is for the outside user button

// define the function to detect the touch screen events
void detect_touch_events(); // this is for the touch screen events

// gyroscope functions
void init_gyroscope(); // define the function to initialize the gyroscope
void read_gryscope(); // define the function to read the gyroscope data

// define the function to record and attempt the sequence
void record_sequence(); // define the function to record the sequence
void attempt_sequence(); // define the function to attempt the sequence
// define the function to compare the recorded and attempted sequence
float compare_sequence(std::vector<std::vector<float>> &seq1, std::vector<std::vector<float>> &seq2);

// define the function to lcd display
void lcd_display_function(); // define the function to display the lcd information
void setup_background_layer(); // define the function to setup the background layer
void lcd_display_design(); // define the function to display the lcd design
void clear_screen();  // define the function to clear the screen

// define the functions to process the sequence
// define the function to calibrate the start position of the sequence
void calibrateStartPosition(std::vector<std::vector<float>>& sequence);
// define the function to calibrate the sequence to remove the noise
void calibrateSequence(std::vector<std::vector<float>>& sequence, float noiseThreshold);
// define the function to normalize the sequence
void normalize_sequence(std::vector<std::vector<float>> &sequence);
// define the function to apply the moving average filter to the sequence
void average_filter(std::vector<std::vector<float>> &sequence, int window_size);
// define the function to apply the Guassian filter to the sequence
std::vector<float> computeGaussianKernel(int kernelSize, float sigma);
std::vector<float> applyGaussianFilter(std::vector<float>& sequence, std::vector<float>& kernel);
std::vector<std::vector<float>> applyGaussianFilter2D(std::vector<std::vector<float>>& sequence2D, std::vector<float>& kernel);

// define the function to clear the sequence when it is not needed
void clear_sequence(std::vector<std::vector<float>> &sequence);

//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

void data_cb(){
  flags.set(DATA_READY_FLAG);
};

void button1_handler() {
  // set the button event to be record
  button_event.set(TS_FLAG_Record);
  // button_event.set(TS_FLAG_Record);
  button1_pressed = true;
}

void button2_handler() {
  // set the button event to be attempt
  button_event.set(TS_FLAG_Attempt);
  button2_pressed = true;
}
void button2_handler2() {
  button_event.set(TS_FLAG_Attempt);
}
void detect_touch_events(){
  // detect the touch screen events
  ts.GetState(&TS_State);
  // we want to expire the range once it get pressed 

  if (TS_State.TouchDetected) {
    uint16_t x = TS_State.X; 
    uint16_t y = 320 - TS_State.Y;
    // if the touch screen is pressed, we will record the sequence
    if (x >= BUTTON1_X_START && x <= BUTTON1_X_END && y >= BUTTON1_Y_START && y <= BUTTON1_Y_END) {
      if(!button1_pressed){
        button1_handler();
      }
    }
    // Check if the touch event occurred within the button2 area
    if (x >= BUTTON2_X_START && x <= BUTTON2_X_END && y >= BUTTON2_Y_START && y <= BUTTON2_Y_END) {
      if(!button2_pressed){
        button2_handler();
      }
      if (flags.get()&Locked_Flag && button2_pressed){
        flags.clear(Locked_Flag);
        button_event.set(Record_Flag);
        button2_handler2();
        ThisThread::sleep_for(100ms);
      }
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
  // // print raw x,y,x
  // printf("raw_gx: %d, raw_gy: %d, raw_gz: %d\n",raw_gx,raw_gy,raw_gz);
  // printf("raw_gx: %d, raw_gy\n",raw_gx);

  // 17.5f*0.017453292519943295769236907684886f / 1000.0f = 0.000305432619f for 500dps
  // 0.0001527163f for 245dps 
  // here we choose 245dps to increase the sensitivity
  gx=raw_gx*0.0001527163f;//0.000305432619f;
  gy=raw_gy*0.0001527163f;//0.000305432619f;
  gz=raw_gz*0.0001527163f;//0.000305432619f;
  //blink the led
  led1=!led1;
  wait_us(100);
}

// record the sequence of the gyroscope data x,y and z
void record_sequence(){
  for(int i=0;i<MAX_SEQUENCE;i++){
    // printf("Recording sequence %d\n",i);
    read_gryscope();
    recorded_sequence[i][0] = gx;
    recorded_sequence[i][1] = gy;
    recorded_sequence[i][2] = gz;
    ThisThread::sleep_for(100ms);
    recorded_sequences.push_back({recorded_sequence[i][0],recorded_sequence[i][1],recorded_sequence[i][2]});
  }
}
// attempt the sequence of the gyroscope data x,y and z
void attempt_sequence(){
  for(int i=0;i<MAX_SEQUENCE;i++){
    // printf("Attempting sequence %d\n",i);
    read_gryscope();
    attempted_sequence[i][0] = gx;
    attempted_sequence[i][1] = gy;
    attempted_sequence[i][2] = gz;
    ThisThread::sleep_for(100ms);
    attempted_sequences.push_back({attempted_sequence[i][0],attempted_sequence[i][1],attempted_sequence[i][2]});
  }
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

// gryoscope initialization the first place
void calibrateStartPosition(std::vector<std::vector<float>>& sequence) {
  if (sequence.empty()) return;

  // Take the first reading as the initial position
  std::vector<float> initial_position = sequence[0];

  // Subtract the initial position from each data point
  for (auto& dataPoint : sequence) {
    for (size_t i = 0; i < dataPoint.size(); ++i) {
      dataPoint[i] -= initial_position[i];
    }
  }
}
// gryoscope calibration to remove the noise
void calibrateSequence(std::vector<std::vector<float>>& sequence, float noiseThreshold) {
  // Calculate the mean value for each dimension
  std::vector<float> means(sequence[0].size(), 0.0f);
  for (const auto& dataPoint : sequence) {
    for (size_t i = 0; i < dataPoint.size(); ++i) {
      means[i] += dataPoint[i];
    }
  }
  for (auto& mean : means) {
    mean /= sequence.size();
  }

  // Subtract the mean value from each data point and remove noise
  for (auto& dataPoint : sequence) {
    for (size_t i = 0; i < dataPoint.size(); ++i) {
      dataPoint[i] -= means[i];
      if (std::abs(dataPoint[i]) < noiseThreshold) {
        dataPoint[i] = 0.0f;
      }
    }
  }
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

/*
apply Guassian filter to the recorded or attempted sequence
try to implement Gaussian Filter here
Function to compute Gaussian kernel
*/
std::vector<float> computeGaussianKernel(int kernelSize, float sigma) {
  std::vector<float> kernel(kernelSize);
  float sum = 0.0;
  int midpoint = kernelSize / 2;

  for (int i = 0; i < kernelSize; ++i) {
    int x = i - midpoint;
    kernel[i] = std::exp(-0.5 * (x * x) / (sigma * sigma));
    sum += kernel[i];
  }

  for (int i = 0; i < kernelSize; ++i) {
    kernel[i] /= sum;
  }
  return kernel;
}

// Function to apply Gaussian filter to a sequence
std::vector<float> applyGaussianFilter(std::vector<float>& sequence, std::vector<float>& kernel) {
  int kernelSize = kernel.size();
  int sequenceSize = sequence.size();
  int padding = kernelSize / 2;

  // Create a padded sequence to handle edges
  std::vector<float> paddedSequence(sequenceSize + 2 * padding);
  for (int i = 0; i < sequenceSize; ++i) {
    paddedSequence[i + padding] = sequence[i];
  }

  // Apply Gaussian filter
  std::vector<float> filteredSequence(sequenceSize);
  for (int i = 0; i < sequenceSize; ++i) {
    float sum = 0.0;
    for (int j = 0; j < kernelSize; ++j) {
      sum += kernel[j] * paddedSequence[i + j];
    }
    filteredSequence[i] = sum;
  }

  return filteredSequence;
}

// Function to apply Gaussian filter to a 2D sequence
std::vector<std::vector<float>> applyGaussianFilter2D(std::vector<std::vector<float>>& sequence2D, std::vector<float>& kernel) {
  int sequenceSize = sequence2D.size();

  // Apply Gaussian filter to each sequence in 2D sequence
  std::vector<std::vector<float>> filteredSequence2D(sequenceSize);
  for (int i = 0; i < sequenceSize; ++i) {
    filteredSequence2D[i] = applyGaussianFilter(sequence2D[i], kernel);
  }

  return filteredSequence2D;
}

// compare the recorded sequence and attempted sequence using DTW algorithm
float compare_sequence(std::vector<std::vector<float>> &seq1, std::vector<std::vector<float>> &seq2){
  // use the DTW algorithm to compare the two sequences
  // here is for the normalization
  // normalize the two sequences
  normalize_sequence(seq1);
  normalize_sequence(seq2);
  
  // calibration the start position of the two sequences
  calibrateStartPosition(seq1);
  calibrateSequence(seq1, 0.0001);
  calibrateStartPosition(seq2);
  calibrateSequence(seq2, 0.0001);

  // here is for the moving average filter
  // // apply the moving average filter to the two sequences
  average_filter(seq1, 5);
  average_filter(seq2, 5);

  // here is for the Guassian filter
  // apply the guassian filter to the two sequences
  // int kernelSize = 3;  // adjust as needed
  // float sigma = 1.0;  // adjust as needed
  // std::vector<float> kernel = computeGaussianKernel(kernelSize, sigma);
  // seq1 = applyGaussianFilter2D(seq1, kernel);
  // seq2 = applyGaussianFilter2D(seq2, kernel);
  // apply the DTW algorithm to the two sequences
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
  // printf("dtw[m][n]: %d\n",int(dtw[m][n]));
  return dtw[m][n];
  

}
// clear part area of the screen
void clear_screen()
{
  // ThisThread::sleep_for(1000ms);
  int startX = 10; // X-coordinate of the top-left corner
  int startY = 76; // Y-coordinate of the top-left corner
  int width = 220; // Width of the area to be cleared
  int height = 65; // Height of the area to be cleared
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_BLACK);
  lcd.FillRect(startX, startY, width, height);
  lcd.SetTextColor(LCD_COLOR_GREEN); // set color back to green
}
// define the LCD callback function to real-time display the LCD information
void lcd_display_function(){

  while(true){
    // if the button1 is pressed, showing the "recording" information
    if(button_event.get()&TS_FLAG_Record){
      lcd.DisplayStringAt(10, 160, (uint8_t *)" Recording... ", CENTER_MODE);
      //reset the button event
      button_event.clear(TS_FLAG_Record);
      // set the record flag to record the sequence
      flags.set(Record_Flag);
    }
    // once the recording is finished, convert the button information to "Record" 
    if(flags.get()& TS_FLAG_Record){
      lcd.DisplayStringAt(10, 160, (uint8_t *)"Record Button", CENTER_MODE);
      lcd.DisplayStringAt(10, 60, (uint8_t *)" Recorded!", LEFT_MODE);
      flags.clear(TS_FLAG_Record);
      button_event.set(Record_Flag);
    }
    // if the button2 is pressed, showing the "attempting" information
    if(button_event.get()&TS_FLAG_Attempt){
      lcd.DisplayStringAt(10, 240, (uint8_t *)"   Attempting... ", CENTER_MODE);
      //reset the button event
      button_event.clear(TS_FLAG_Attempt);
      // set the attempt flag to attempt the sequence
      flags.set(Attempt_Flag);
    }
    // once the attempting is finished, conver the button information to "Attempt"
    if(flags.get() & TS_FLAG_Attempt){
      lcd.DisplayStringAt(10, 240, (uint8_t *)"Attempt Button", CENTER_MODE);
      lcd.DisplayStringAt(10, 76, (uint8_t *)" Attempting", LEFT_MODE);
      flags.clear(TS_FLAG_Attempt);
      button_event.set(Attempt_Flag);
    }
    // if the record and attempt button is pressed, showing the "comparing" information
    // also the record and attempt sequence should not be empty
    if(button_event.get()&Record_Flag && button_event.get()&Attempt_Flag && !recorded_sequences.empty() && !attempted_sequences.empty()){
      flags.set(Compare_Flag);
      
      lcd.DisplayStringAt(10, 92, (uint8_t *)" Start Comparing... ", LEFT_MODE);
      
      button_event.clear(Record_Flag);
      button_event.clear(Attempt_Flag);
      
    }

    // if the similiarity result is get, showing the information "Compare Done"
    if (button_event.get()&Compare_Done_Flag){
      button_event.clear(Compare_Done_Flag);
      lcd.DisplayStringAt(10, 108, (uint8_t *)" Compare Done  ", LEFT_MODE);
      // print the lock sucessful or failed based on the similarity
      // set the text color to be red
      lcd.SetTextColor(LCD_COLOR_RED);
      if(similiarity<similiarity_threshold){
        lcd.DisplayStringAt(10, 125, (uint8_t *)" Unlock Success!", LEFT_MODE);
        lcd.SetTextColor(LCD_COLOR_CYAN);
        lcd.DisplayStringAt(10, 300, (uint8_t *)" SUCCESSFUL!",CENTER_MODE);
        lcd.SetTextColor(LCD_COLOR_GREEN);
        // lcd.DisplayStringAt(10, 240, (uint8_t *)"Unlock Success!", CENTER_MODE);
        flags.set(Unlocked_Flag);
      }
      else{
        lcd.DisplayStringAt(10, 125, (uint8_t *)" Unlock Failed!", LEFT_MODE);
        lcd.SetTextColor(LCD_COLOR_CYAN);
        lcd.DisplayStringAt(10, 300, (uint8_t *)"  FAILED!  ", CENTER_MODE);
        lcd.SetTextColor(LCD_COLOR_GREEN);
        // clear_screen();
        flags.set(Locked_Flag);
      }
      // set the text color to be green back
      lcd.SetTextColor(LCD_COLOR_GREEN);
      lcd.DisplayStringAt(10, 240, (uint8_t *)"Attempt Button", CENTER_MODE);
      // display the attempt times with the variable attempt_time

      if (attempt_time <=attempted_size){
        char message[30];
        sprintf(message, "Already Tried %d", attempt_time);
        lcd.SetTextColor(LCD_COLOR_YELLOW);
        lcd.DisplayStringAt(10, 280, (uint8_t *)message, CENTER_MODE);
        lcd.SetTextColor(LCD_COLOR_GREEN);
      }
    }
    if(button_event.get()&Attempt_Limit_Flag){
      
      lcd.SetTextColor(LCD_COLOR_YELLOW);
      if(similiarity > similiarity_threshold){
        lcd.DisplayStringAt(10, 280, (uint8_t *)"Try again later", CENTER_MODE);
      }
      else{
        lcd.DisplayStringAt(10, 280, (uint8_t *)"Finally it works!", CENTER_MODE);
      }
      lcd.SetTextColor(LCD_COLOR_GREEN);
      button_event.clear(Attempt_Limit_Flag);
      // flags.set(Compare_Done_Flag);
      flags.set(Attempt_Limit_Flag);
    }

    ThisThread::sleep_for(10ms);
  }
}

// create the lcd display setting 
void lcd_display_design(){
  // initialize the lcd
  setup_background_layer();
  // Display the title on the screen
  lcd.DisplayStringAt(8, 20, (uint8_t *)"2023 Gesture Unlock", CENTER_MODE);
  // lcd.DisplayStringAt(10, 50, (uint8_t *)"RTES Project", CENTER_MODE);
  lcd.DisplayStringAt(10, 40, (uint8_t *)"Team 40", CENTER_MODE);
  // lcd.DisplayStringAt(10, 90, (uint8_t *)"zw3464", CENTER_MODE);
  // Display the buttons on the screen
  lcd.DisplayStringAt(10, 160, (uint8_t *)"Record Button", CENTER_MODE);
  lcd.DisplayStringAt(10, 240, (uint8_t *)"Attempt Button", CENTER_MODE);

  // Display the button rectuangles on the screen
  lcd.DrawRect(30, 145, 190, 50);
  lcd.DrawRect(30, 225, 190, 50);
  
}

// clear 2d vector
void clear_2d_vector(std::vector<std::vector<float>>& vec) {
  for (auto& innerVec : vec) {
    innerVec.clear();
  }
  vec.clear();
}
void outside_button_handler(){
  // clear the screen log
  clear_flag = true;
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
  
  // define the lcd thread
  lcd_thread.start(lcd_display_function);
  // setting the lcd display
  lcd_display_design();
  // define the user button interrupt
  button.fall(&outside_button_handler);

  while (1) {
    // detect if the screen area is pressed
    detect_touch_events();
    if(attempt_time > attempted_size){
      button_event.set(Attempt_Limit_Flag);
    }
    if(flags.get()&Attempt_Limit_Flag){
      flags.clear(Attempt_Limit_Flag);
      printf("you already try 4 times, please try again later\n");
      return 0;
    }

    // if the record flag is set, record the sequence
    if(flags.get()&Record_Flag){
      // record the sequence
      record_sequence();
      // reset the record flag
      flags.clear(Record_Flag);
      flags.set(TS_FLAG_Record);
      printf("Recorded the sequence\n");
    }
    // if the attempt flag is set, attempt the sequence
    if(flags.get()&Attempt_Flag){
      // attempt the sequence
      clear_2d_vector(attempted_sequences);
      attempt_sequence();
      // reset the attempt flag
      flags.clear(Attempt_Flag);
      flags.set(TS_FLAG_Attempt);
      attempt_time ++;
    }
    // here we want to design the user to try 4 times,
    // if the user failed to unlock the device for 3 times, we will lock the device
    // if the record and attempt flag is set, compare the sequence
    if(flags.get() & Compare_Flag){
      // compare the sequence
      similiarity = compare_sequence(recorded_sequences,attempted_sequences);
      printf("Attempting the sequence %d\n", int(attempt_time));
      printf("similiarity: %d\n",int(similiarity));
      // set the compare flag
      flags.clear(Compare_Flag);
      button_event.set(Compare_Done_Flag);
    }
    // if the unlocking is correct, anything need to do
    if(flags.get() & Unlocked_Flag){
      printf("Unlock Success!\n");
      flags.clear(Unlocked_Flag);
      // return 0;
    }
    if(clear_flag){
      clear_flag = false;
      clear_screen();
    }
    // if the unlocking is failed, we will lock the device
    // this part is done in the detect touch screen part 
    // since it will only happen when the current unlocking is not successful
  }
}
