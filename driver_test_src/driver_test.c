/* File: driver_test.c
 * Author: TR
 * Description: Test program for i2s_driver
 * Version History
 * v1.0 Initial release
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>

/* IOCTL commands copied from the i2s_driver header */
#define I2S_SET_EN          _IOW('i', 0, char)
#define I2S_SET_TXON        _IOW('i', 1, char)
#define I2S_SET_RXON        _IOW('i', 2, char)
#define I2S_TX_BUFF_SPACE   _IOR('i', 3, int)
#define I2S_RX_BUFF_ITEMS   _IOR('i', 4, int)
#define I2S_CLEAR_TX_BUFF   _IOW('i', 5, char)
#define I2S_CLEAR_RX_BUFF   _IOW('i', 6, char)
#define I2S_WRITE_CS_A      _IOW('i', 7, uint32_t)
#define I2S_WRITE_MODE_A    _IOW('i', 8, uint32_t)
#define I2S_WRITE_RXC_A     _IOW('i', 9, uint32_t)
#define I2S_WRITE_TXC_A     _IOW('i', 10, uint32_t)
#define I2S_WRITE_DREQ_A    _IOW('i', 11, uint32_t)
#define I2S_WRITE_INTEN_A   _IOW('i', 12, uint32_t)
#define I2S_WRITE_INTSTC_A  _IOW('i', 13, uint32_t)
#define I2S_WRITE_GRAY      _IOW('i', 14, uint32_t)
#define I2S_CLR_TX_FIFO     _IOW('i', 15, char)
#define I2S_CLR_RX_FIFO     _IOW('i', 16, char)

#define RECEIVED_DATA_SIZE        1024
#define RECEIVED_DATA_SIZE_BYTES  4 * RECEIVED_DATA_SIZE

int main(int argc, char **argv)
{
  int i, j = 0;
  size_t nbytes, transfer_length;

  int wav_samples, bytes_per_sample;
  int32_t temp_wav;
  int32_t *wav_data;
  unsigned int wav_data_offset = 0;
  int samples_available;

  int32_t received_data[RECEIVED_DATA_SIZE];
  int buffer_space, buffer_size;

  // Open i2s dev file
  printf("Opening i2s file...\n");
  int i2s_fd = open("/dev/i2s", O_RDWR);

  ioctl(i2s_fd, I2S_SET_TXON, 0);   // Make sure tx is disabled
  /* Clear software buffers */
  ioctl(i2s_fd, I2S_CLEAR_TX_BUFF, 0);
  ioctl(i2s_fd, I2S_CLEAR_RX_BUFF, 0);
  /* Clear hardware FIFOs */
  ioctl(i2s_fd, I2S_CLR_TX_FIFO, 0);
  ioctl(i2s_fd, I2S_CLR_RX_FIFO, 0);

  nbytes = sizeof(int32_t);

  printf("Running TX example.\n");

  if(argc < 2)
  {
    printf("Please provide a .wav file.\n");
    return -1;
  }

  int sound_fp = open(argv[1], O_RDONLY);

  // Get max buffer size before the buffer is filled
  buffer_size = ioctl(i2s_fd, I2S_TX_BUFF_SPACE);
  printf("Kernel buffer size: %d\n", buffer_size);

  /* Read the number of bits per sample at offset 34 */
  pread(sound_fp, &bytes_per_sample, 2, 34);
  bytes_per_sample = (bytes_per_sample & 0xFFFF) / 8;    // Convert to bytes
  printf("Bytes per sample: %d\n", bytes_per_sample);

  /* Read the number of samples in the file located at byte offset 40 */
  pread(sound_fp, &wav_samples, 4, 40);
  printf("Number of samples in wav file: %d\n", wav_samples);

  //wav_samples = 20000000l; 
  /* Create a huge array to hold all the samples */
  /* NOTE: Probably don't do this for large file sizes */
  wav_data = (int32_t *) malloc(wav_samples * sizeof(int32_t));

  /* Copy data from wav file into memory */
  for(i = 0; i < wav_samples; i++)
  {
    /* Audio data starts at offset 44 */
    pread(sound_fp, &temp_wav, bytes_per_sample, 44 + bytes_per_sample*i);
    temp_wav = temp_wav << (8 * (4 - bytes_per_sample)); 
    //temp_wav =  temp_wav & 0xFFFF;
    *(wav_data + i) = (int32_t) temp_wav;
    if(i < 1000) {
	    if(i%16 == 0) {
		printf("\n");
	    }
	    printf("%x ", (int32_t)temp_wav);
    }
  }

  // Initialization of buffer
  for(i = 0; i < buffer_size - 1; ++i)
  {
    //printf("wav_data_offset = %d\n", wav_data_offset);
    if(ioctl(i2s_fd, I2S_TX_BUFF_SPACE) == 0)
    {
      printf("Buffer is full.\n");
      printf("i = %d\n", i);
      break;
    }
    write(i2s_fd, wav_data + wav_data_offset, nbytes);
    wav_data_offset++;
  }

  // Enable TX
  printf("Enabling TX...\n");
  usleep(5);
  ioctl(i2s_fd, I2S_SET_TXON, 1);

  usleep(1);

  while(wav_data_offset < wav_samples)    // Loop until the file is over
  {
    //printf("wav_data_offset = %d\n", wav_data_offset);  
    usleep(1);
    buffer_space = ioctl(i2s_fd, I2S_TX_BUFF_SPACE);
    for(j = 0; j < buffer_space; j++)
    {
      //printf("wav_data_offset = %d\n", wav_data_offset); 
      if(wav_data_offset >= wav_samples)
      {
        break;
      }
      write(i2s_fd, wav_data + wav_data_offset, nbytes);
      wav_data_offset++;
    }
  }

  printf("Disabling TX...\n");
  ioctl(i2s_fd, I2S_SET_TXON, 0);

  free(wav_data);
  close(sound_fp);

  printf("Closing files.\n");
  close(i2s_fd);

  return 0;
}
