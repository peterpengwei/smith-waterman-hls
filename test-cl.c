/*******************************************************************************
Vendor: Xilinx 
Associated Filename: test-cl.c
Purpose: OpenCL Host Code for Matrix Multiply Example
Revision History: July 1, 2013 - initial release
                                                
*******************************************************************************
Copyright (C) 2013 XILINX, Inc.

This file contains confidential and proprietary information of Xilinx, Inc. and 
is protected under U.S. and international copyright and other intellectual 
property laws.

DISCLAIMER
This disclaimer is not a license and does not grant any rights to the materials 
distributed herewith. Except as otherwise provided in a valid license issued to 
you by Xilinx, and to the maximum extent permitted by applicable law: 
(1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX 
HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, 
INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR 
FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether 
in contract or tort, including negligence, or under any other theory of 
liability) for any loss or damage of any kind or nature related to, arising under 
or in connection with these materials, including for any direct, or any indirect, 
special, incidental, or consequential loss or damage (including loss of data, 
profits, goodwill, or any type of loss or damage suffered as a result of any 
action brought by a third party) even if such damage or loss was reasonably 
foreseeable or Xilinx had been advised of the possibility of the same.

CRITICAL APPLICATIONS
Xilinx products are not designed or intended to be fail-safe, or for use in any 
application requiring fail-safe performance, such as life-support or safety 
devices or systems, Class III medical devices, nuclear facilities, applications 
related to the deployment of airbags, or any other applications that could lead 
to death, personal injury, or severe property or environmental damage 
(individually and collectively, "Critical Applications"). Customer assumes the 
sole risk and liability of any use of Xilinx products in Critical Applications, 
subject only to applicable laws and regulations governing limitations on product 
liability. 

THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT 
ALL TIMES.

*******************************************************************************/
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <CL/opencl.h>

#include <sys/time.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////
typedef struct timespec timespec;
timespec diff(timespec start, timespec end);
timespec sum(timespec t1, timespec t2);
void printTimeSpec(timespec t);
timespec tic( );
void toc( timespec* start_time );
double get_current_msec( );

timespec diff(timespec start, timespec end)
{   
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

timespec sum(timespec t1, timespec t2) {
	timespec temp;
	if (t1.tv_nsec + t2.tv_nsec >= 1000000000) {
		temp.tv_sec = t1.tv_sec + t2.tv_sec + 1;
		temp.tv_nsec = t1.tv_nsec + t2.tv_nsec - 1000000000;
	} else {
		temp.tv_sec = t1.tv_sec + t2.tv_sec;
		temp.tv_nsec = t1.tv_nsec + t2.tv_nsec;
	}
	return temp;
}

void printTimeSpec(timespec t) {
	printf("elapsed time: %d.%09d\n", (int)t.tv_sec, (int)t.tv_nsec);
}

void printTimeSpec_wrbuf(timespec t) {
	printf("write DRAM time: %d.%09d\n", (int)t.tv_sec, (int)t.tv_nsec);
}

timespec tic( )
{
	timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

void toc( timespec* start_time )
{
	timespec current_time;
	clock_gettime(CLOCK_REALTIME, &current_time);
	printTimeSpec( diff( *start_time, current_time ) );
	*start_time = current_time;
}

void toc_wrbuf( timespec* start_time )
{
	timespec current_time;
	clock_gettime(CLOCK_REALTIME, &current_time);
	printTimeSpec_wrbuf( diff( *start_time, current_time ) );
	*start_time = current_time;
}

double get_current_msec( )
{
	timespec current_time;
	clock_gettime(CLOCK_REALTIME, &current_time);
	return current_time.tv_nsec/1e6+current_time.tv_sec*1e3;
}

// Use a static matrix for simplicity
//
#define MATRIX_RANK 64
//#define DATA_SIZE MATRIX_RANK*MATRIX_RANK
#define TASK_NUM 32
//#define DATA_SIZE 102400
//#define RESULT_SIZE 24576
#define DATA_SIZE 8192000
#define TOTAL_TASK_NUMS 262144
#define RESULT_SIZE TOTAL_TASK_NUMS*5


////////////////////////////////////////////////////////////////////////////////

int
load_file_to_memory(const char *filename, char **result)
{ 
  int size = 0;
  FILE *f = fopen(filename, "rb");
  if (f == NULL) 
  { 
    *result = NULL;
    return -1; // -1 means file opening fail 
  } 
  fseek(f, 0, SEEK_END);
  size = ftell(f);
  fseek(f, 0, SEEK_SET);
  *result = (char *)malloc(size+1);
  if (size != fread(*result, sizeof(char), size, f)) 
  { 
    free(*result);
    return -2; // -2 means file reading fail 
  } 
  fclose(f);
  (*result)[size] = 0;
  return size;
}

int main(int argc, char** argv)
{
  int err;                            // error code returned from api calls
     
  //int a[DATA_SIZE];                   // original data set given to device
  int *a = (int *) malloc(sizeof(int)*DATA_SIZE);
  //int b[DATA_SIZE];                   // original data set given to device
  //int results[RESULT_SIZE];             // results returned from device
  int *results = (int *) malloc(sizeof(int)*RESULT_SIZE);
  //int sw_results[DATA_SIZE];          // results returned from device
  unsigned int correct;               // number of correct results returned

  size_t global[2];                   // global domain size for our calculation
  size_t local[2];                    // local domain size for our calculation

  cl_platform_id platform_id;         // platform id
  cl_device_id device_id;             // compute device id 
  cl_context context;                 // compute context
  cl_command_queue commands;          // compute command queue
  cl_program program;                 // compute program
  cl_kernel kernel;                   // compute kernel
   
  char cl_platform_vendor[1001];
  char cl_platform_name[1001];
   
  cl_mem input_a;                     // device memory used for the input array
  //cl_mem input_b;                     // device memory used for the input array
  cl_mem output_a;                      // device memory used for the output array
  int inc;
  double t_start, t_end;

  int size = 0;
  FILE *f_inp = fopen("/curr/jielei/cmost_lib_flow/picasso_flow/Test_Data/total_input.dat", "rb");
  FILE *f_outp = fopen("results.dat", "wb");

  int seed_index=0;
  int *resul_reorder = (int *) malloc(sizeof(int)*TOTAL_TASK_NUMS*4);

//  int test_inp[DATA_SIZE];
  if (f_inp == NULL) 
  { 
	 return -1; // -1 means file opening fail 
  } 
  fseek(f_inp, 0, SEEK_END);
  size = ftell(f_inp);
  fseek(f_inp, 0, SEEK_SET);
  size = (size+3)/4;
  printf("size = %d\n", size);
  
  if (argc != 2){
    printf("%s <inputfile>\n", argv[0]);
    return EXIT_FAILURE;
  }

  // Fill our data sets with pattern
  //

  int i=0, j=0;
  for(i = 0; i < size; i++) {
  //for(i = 0; i < 642; i++) {
	//  a[i] = test_inp[i];
    fread(&a[i], sizeof(int), 1, f_inp);
    //b[i] = (int)i;
//    results[i] = 0;
  }
  inc = a[2];
   printf("inc = %d\n", inc);
  for(i = 0; i < inc*5; i++) {
	  results[i] = 0;
  }

  // Connect to first platform
  //
  err = clGetPlatformIDs(1,&platform_id,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to find an OpenCL platform!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  err = clGetPlatformInfo(platform_id,CL_PLATFORM_VENDOR,1000,(void *)cl_platform_vendor,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: clGetPlatformInfo(CL_PLATFORM_VENDOR) failed!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  printf("CL_PLATFORM_VENDOR %s\n",cl_platform_vendor);
  err = clGetPlatformInfo(platform_id,CL_PLATFORM_NAME,1000,(void *)cl_platform_name,NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: clGetPlatformInfo(CL_PLATFORM_NAME) failed!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  printf("CL_PLATFORM_NAME %s\n",cl_platform_name);
 
  // Connect to a compute device
  //
  int fpga = 0;
#if defined (FPGA_DEVICE)
  fpga = 1;
#endif
  err = clGetDeviceIDs(platform_id, fpga ? CL_DEVICE_TYPE_ACCELERATOR : CL_DEVICE_TYPE_CPU,
                       1, &device_id, NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to create a device group!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  
  // Create a compute context 
  //
  context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);
  if (!context)
  {
    printf("Error: Failed to create a compute context!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }

  // Create a command commands
  //
  commands = clCreateCommandQueue(context, device_id, 0, &err);
  if (!commands)
  {
    printf("Error: Failed to create a command commands!\n");
    printf("Error: code %i\n",err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }

  int status;

  // Create Program Objects
  //
  
  // Load binary from disk
  unsigned char *kernelbinary;
  char *xclbin=argv[1];
  printf("loading %s\n", xclbin);
  int n_i = load_file_to_memory(xclbin, (char **) &kernelbinary);
  if (n_i < 0) {
    printf("failed to load kernel from xclbin: %s\n", xclbin);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to load kernel from xclbin: %s\n", xclbin);
  }
  size_t n = n_i;
  // Create the compute program from offline
  program = clCreateProgramWithBinary(context, 1, &device_id, &n,
                                      (const unsigned char **) &kernelbinary, &status, &err);
  if ((!program) || (err!=CL_SUCCESS)) {
    printf("Error: Failed to create compute program from binary %d!\n", err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to create compute program from binary %d!\n", err);
  }

  // Build the program executable
  //
  err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
  if (err != CL_SUCCESS)
  {
    size_t len;
    char buffer[2048];

    printf("Error: Failed to build program executable!\n");
    clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
    printf("%s\n", buffer);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to build program executable!\n");
  }

  // Create the compute kernel in the program we wish to run
  //
  kernel = clCreateKernel(program, "mmult", &err);
  if (!kernel || err != CL_SUCCESS)
  {
    printf("Error: Failed to create compute kernel!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  else {
	  printf("Succeed to create compute kernel!\n");
  }

  // Create the input and output arrays in device memory for our calculation
  //
  //input_a = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * DATA_SIZE, NULL, NULL);
  input_a = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * size, NULL, NULL);
  //input_b = clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(int) * DATA_SIZE, NULL, NULL);
  //output = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(int) * RESULT_SIZE, NULL, NULL);
  output_a = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(int) * (inc*5), NULL, NULL);
  //if (!input_a || !input_b || !output)
  if (!input_a || !output_a)
  {
    printf("Error: Failed to allocate device memory!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }    
  else {
	  printf("Succeed to allocate device memory!\n");
  }

  // Get the start time
  timespec timer = tic( );
    
  // Write our data set into the input array in device memory 
  //
  //err = clEnqueueWriteBuffer(commands, input_a, CL_TRUE, 0, sizeof(int) * DATA_SIZE, a, 0, NULL, NULL);
  err = clEnqueueWriteBuffer(commands, input_a, CL_TRUE, 0, sizeof(int) * size, a, 0, NULL, NULL);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to write to source array a!\n");
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  //else {
	//  printf("Succeed to write to source array a!\n");
  //}

  // Write our data set into the input array in device memory 
  //
  //err = clEnqueueWriteBuffer(commands, input_b, CL_TRUE, 0, sizeof(int) * DATA_SIZE, b, 0, NULL, NULL);
  //if (err != CL_SUCCESS)
  //{
  //  printf("Error: Failed to write to source array b!\n");
  //  printf("Test failed\n");
  //  return EXIT_FAILURE;
  //}
    
  // Set the arguments to our compute kernel
  //
  err = 0;
  err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input_a);
  //err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &input_b);
  err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output_a);
  err |= clSetKernelArg(kernel, 2, sizeof(int), &inc);
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to set kernel arguments! %d\n", err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }

  toc_wrbuf(&timer);
  //else {
	//  printf("Succeed to set kernel arguments! %d\n", err);
  //}

  // Execute the kernel over the entire range of our 1d input data set
  // using the maximum number of work group items for this device
  //

#ifdef C_KERNEL
  err = clEnqueueTask(commands, kernel, 0, NULL, NULL);
#else
  //global[0] = MATRIX_RANK;
  //global[1] = MATRIX_RANK;
  //local[0] = MATRIX_RANK;
  //local[1] = MATRIX_RANK;
  global[0] = DATA_SIZE;
  global[1] = DATA_SIZE;
  local[0] = DATA_SIZE;
  local[1] = DATA_SIZE;
 err = clEnqueueNDRangeKernel(commands, kernel, 2, NULL, 
                               (size_t*)&global, (size_t*)&local, 0, NULL, NULL);
#endif
  if (err)
  {
    printf("Error: Failed to execute kernel! %d\n", err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  //else {
  //	  printf("Succeed to execute kernel! %d\n", err);
  //}

  // Read back the results from the device to verify the output
  //
  cl_event readevent;
  //err = clEnqueueReadBuffer( commands, output, CL_TRUE, 0, sizeof(int) * RESULT_SIZE, results, 0, NULL, &readevent );  
  err = clEnqueueReadBuffer( commands, output_a, CL_TRUE, 0, sizeof(int) * (inc*5), results, 0, NULL, &readevent );  
  if (err != CL_SUCCESS)
  {
    printf("Error: Failed to read output_a array! %d\n", err);
    printf("Test failed\n");
    return EXIT_FAILURE;
  }
  //else {
  //	  printf("Succeed to read output array! %d\n", err);
  //}

  clWaitForEvents(1, &readevent);

  // Get the execution time
  toc(&timer);

  printf("results[0] = %d\n", results[0]);


  for (i=0;i<inc;i++) {
	  seed_index = results[i*5];
	  for (j=0; j<4; j++) {
		  resul_reorder[seed_index*4 + j] = results[i*5 + j + 1];
	  }
	  //fwrite(&results[i], sizeof(int), 1, f_outp);
  }

  for (i=0; i<inc*4; i++) {
	  fwrite(&resul_reorder[i], sizeof(int), 1, f_outp);
  }
 //printf("A\n");
  //for (i=0;i<DATA_SIZE;i++) {
  //  printf("%x ",a[i]);
  //  if (((i+1) % 16) == 0)
  //    printf("\n");
  //}
  //printf("B\n");
  //for (i=0;i<DATA_SIZE;i++) {
  //  printf("%x ",b[i]);
  //  if (((i+1) % 16) == 0)
  //    printf("\n");
  //}
  //printf("res\n");
  //for (i=0;i<DATA_SIZE;i++) {
  //  printf("%x ",results[i]);
  //  if (((i+1) % 16) == 0)
  //    printf("\n");
  //}
    
  // Validate our results
  //
  //correct = 0;
  //for(i = 0; i < DATA_SIZE; i++)
  //{
  //  int row = i/MATRIX_RANK;
  //  int col = i%MATRIX_RANK;
  //  int running = 0;
  //  int index;
  //  for (index=0;index<MATRIX_RANK;index++) {
  //    int aIndex = row*MATRIX_RANK + index;
  //    int bIndex = col + index*MATRIX_RANK;
  //    running += a[aIndex] * b[bIndex];
  //  }
  //  sw_results[i] = running;
  //}
  //  
  //for (i = 0;i < DATA_SIZE; i++) 
  //  if(results[i] == sw_results[i])
  //    correct++;
  //printf("Software\n");
  //for (i=0;i<DATA_SIZE;i++) {
  //  //printf("%0.2f ",sw_results[i]);
  //  printf("%d ",sw_results[i]);
  //  if (((i+1) % 16) == 0)
  //    printf("\n");
  //}
    
    
  // Print a brief summary detailing the results
  //
  //printf("Computed '%d/%d' correct values!\n", correct, DATA_SIZE);
    
  // Shutdown and cleanup
  //
  clReleaseMemObject(input_a);
  //clReleaseMemObject(input_b);
  clReleaseMemObject(output_a);
  clReleaseProgram(program);
  clReleaseKernel(kernel);
  clReleaseCommandQueue(commands);
  clReleaseContext(context);
  free(resul_reorder);
  resul_reorder = NULL;
  fclose(f_inp);
  fclose(f_outp);
  f_inp = NULL;
  f_outp = NULL;

  //if(correct == DATA_SIZE){
  //  printf("Test passed!\n");
    return EXIT_SUCCESS;
  //}
  //else{
  //  printf("Test failed\n");
  //  return EXIT_FAILURE;
  //}
}
