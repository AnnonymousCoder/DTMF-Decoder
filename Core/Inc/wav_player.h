/*
 * wav_player.h
 *
 *  Created on: Nov 24, 2024
 *      Author: matth
 */

#ifndef INC_WAV_PLAYER_H_
#define INC_WAV_PLAYER_H_

#include "fatfs.h"
#include <stdarg.h>

typedef struct WAVE_FILE_HEADER{
	uint32_t ChunkID;
	uint32_t ChunkSize;
	uint32_t Format;
	uint32_t SubChunk_1_ID;
	uint32_t SubChunk_1_Size;
	uint16_t AudioFormat;
	uint16_t NumChannels;
	uint32_t SampleRate;
	uint32_t ByteRate;
	uint16_t BlockAlign;
	uint16_t BitsPerSample;
	uint32_t SubChunk_2_ID;
	uint32_t SubChunk_2_Size;
}WAVE_HEADER;

typedef struct FILE_INSTANCE{
	FATFS 		f_system;
	FIL 		file;
	WAVE_HEADER header;
	uint16_t*	buffer;
	uint32_t	buf_size;
}PLAY_FILE;

typedef enum PLAY_STATE{
	PLAY_ERROR 	= 0x01,
	SD_MOUNT	= 0x02,
	FILE_OPEN	= 0x04,
	PLAY_STATE	= 0x08,
	READ_CMPLT	= 0x10,
	BUFFER_CMPLT	= 0x20,
	CLOSE_FILE		= 0x40
}P_STATE;

#define DEVICE_PATH ""
#define f_unmount(path) f_mount(NULL, path, 0)

PLAY_FILE* __handle;
P_STATE* __state;

extern void myprintf(const char*, ...);

void init_player(const char* file_name, uint16_t* buffer, uint32_t length, PLAY_FILE* pf_handle, P_STATE* state){
	if(!pf_handle)
		return;

	FRESULT res;
	UINT bytes_read;

	HAL_Delay(1000);
	res = f_mount(&(pf_handle->f_system), DEVICE_PATH, 1);
	if(res != FR_OK){
		*state |= PLAY_ERROR;
		return;
	}

	*state |= SD_MOUNT;
	myprintf("Successfully Mounted SD CARD\r\n");

	res = f_open(&(pf_handle->file), file_name, FA_READ);
	if(res != FR_OK){
		f_unmount(DEVICE_PATH);
		*state |= PLAY_ERROR;
		return;
	}

	*state |= FILE_OPEN;
	myprintf("Successfully Opened '%s' file\r\n", file_name);

	res = f_read(&(pf_handle->file), &(pf_handle->header), sizeof(pf_handle->header), &bytes_read);
	if(res != FR_OK){
		f_close(&(pf_handle->file));
		f_unmount(DEVICE_PATH);
		*state |= PLAY_ERROR | CLOSE_FILE;
		return;
	}

	pf_handle->buffer = buffer;
	pf_handle->buf_size = length;
	__handle = pf_handle;
	__state  = state;
}

UINT __play_output_stream_t16(uint16_t* buffer, PLAY_FILE* pf_handle, P_STATE* state){
	if(pf_handle == NULL)
		return (*__state & PLAY_STATE);

	int16_t data = 0;
	UINT cnt = 0, btr = 0;
	FRESULT res;

	do{
		if(cnt >= pf_handle->buf_size){ *state |= BUFFER_CMPLT; break;}

		res = f_read(&(pf_handle->file), &data, 1 * sizeof(data), &btr);
		if(res != FR_OK)
			break;

		*buffer = ((data >> 4) + 0x800) &0xfff;
		buffer++;
		cnt++;
	}while(!f_eof(&(pf_handle->file)) && (*__state & PLAY_STATE));

	if(f_eof(&(pf_handle->file))){
		*state &= ~PLAY_STATE;
		*state |= CLOSE_FILE;
	}else
		*state |= READ_CMPLT;
	return res;
}

void start_player_t16(PLAY_FILE* pf_handle, P_STATE* state){
	if((*state & (PLAY_ERROR | CLOSE_FILE)) > 0)
		return;

	if((*state & (SD_MOUNT | FILE_OPEN)) != (SD_MOUNT | FILE_OPEN))
		return;

	FRESULT res;

	if ((*state & PLAY_STATE) == 0 )
		*state |= PLAY_STATE;

//	myprintf("Player Started.............\r\n");

	res = __play_output_stream_t16(pf_handle->buffer, pf_handle, state);
	if(res != FR_OK){
		f_close(&(pf_handle->file));
		f_unmount(DEVICE_PATH);
		*state |= PLAY_ERROR | CLOSE_FILE;
		return;
	}

//	myprintf("Player Stopping.............");

//	f_close(&(pf_handle->file));
//	f_unmount(DEVICE_PATH);

//	myprintf("Player Stopped.............\r\n");
}

void player_error_handler(PLAY_FILE* pf_handle, P_STATE* state){
	if((*state & (PLAY_ERROR | CLOSE_FILE)) == 0)
		return;

	if((*state & FILE_OPEN) > 0)
		f_close(&(pf_handle->file));

	f_unmount(DEVICE_PATH);
	*state = 0;
	myprintf("File Closed.............\r\n");
}
#endif /* INC_WAV_PLAYER_H_ */
