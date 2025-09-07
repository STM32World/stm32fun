/******************************************************************************
 *  File        : sd_functions.c
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/


#include "fatfs.h"
#include "sd_diskio_spi.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "ffconf.h"

char sd_path[4];
FATFS fs;

int sd_format(void) {
	// Pre-mount required for legacy FatFS
	f_mount(&fs, sd_path, 0);

	FRESULT res;
	//res = f_mkfs(sd_path, 1, 0);
	if (res != FR_OK) {
		printf("Format failed: f_mkfs returned %d\r\n", res);
	}
		return res;
}

int sd_get_space_kb(void) {
	FATFS *pfs;
	DWORD fre_clust, tot_sect, fre_sect, total_kb, free_kb;
	FRESULT res = f_getfree(sd_path, &fre_clust, &pfs);
	if (res != FR_OK) return res;

	tot_sect = (pfs->n_fatent - 2) * pfs->csize;
	fre_sect = fre_clust * pfs->csize;
	total_kb = tot_sect / 2;
	free_kb = fre_sect / 2;
	printf("💾 Total: %lu KB, Free: %lu KB\r\n", total_kb, free_kb);
	return FR_OK;
}

int sd_mount(void) {
	FRESULT res;
	extern uint8_t sd_is_sdhc(void);

	printf("Linking SD driver...\r\n");
	if (FATFS_LinkDriver(&SD_Driver, sd_path) != 0) {
		printf("FATFS_LinkDriver failed\n");
		return FR_DISK_ERR;
	}

	printf("Initializing disk...\r\n");
	DSTATUS stat = disk_initialize(0);
	if (stat != 0) {
		printf("disk_initialize failed: 0x%02X\n", stat);
		printf("FR_NOT_READY\tTry Hard Reset or Check Connection/Power\r\n");
		return FR_NOT_READY;
	}

	printf("Attempting mount at %s...\r\n", sd_path);
	res = f_mount(&fs, sd_path, 1);
	if (res == FR_OK)
	{
		printf("SD card mounted successfully at %s\r\n", sd_path);
		printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

		// Capacity and free space reporting
		sd_get_space_kb();
		return FR_OK;
	}

//	 Handle no filesystem by creating one
	if (res == FR_NO_FILESYSTEM)
	{
		printf("No filesystem found on SD card. Attempting format...\r\nThis will create 32MB Partition (Most Probably)\r\n");
		printf("If you need the full sized SD card, use the computer to format into FAT32\r\n");
		sd_format();

		printf("Retrying mount after format...\r\n");
		res = f_mount(&fs, sd_path, 1);
		if (res == FR_OK) {
			printf("SD card formatted and mounted successfully.\r\n");
			printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

			// Report capacity after format
			sd_get_space_kb();
		}
		else {
			printf("Mount failed even after format: %d\r\n", res);
		}
		return res;
	}

	// Any other mount error
	printf("Mount failed with code: %d\r\n", res);
	return res;
}


int sd_unmount(void) {
	FRESULT res = f_mount(NULL, sd_path, 1);
	printf("SD card unmounted: %s\r\n", (res == FR_OK) ? "OK" : "Failed");
	return res;
}

int sd_write_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Write %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_append_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_lseek(&file, f_size(&file));
	if (res != FR_OK) {
		f_close(&file);
		return res;
	}

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Appended %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read) {
	FIL file;
	*bytes_read = 0;

	FRESULT res = f_open(&file, filename, FA_READ);
	if (res != FR_OK) {
		printf("f_open failed with code: %d\r\n", res);
		return res;
	}

	res = f_read(&file, buffer, bufsize - 1, bytes_read);
	if (res != FR_OK) {
		printf("f_read failed with code: %d\r\n", res);
		f_close(&file);
		return res;
	}

	buffer[*bytes_read] = '\0';

	res = f_close(&file);
	if (res != FR_OK) {
		printf("f_close failed with code: %d\r\n", res);
		return res;
	}

	printf("Read %u bytes from %s\r\n", *bytes_read, filename);
	return FR_OK;
}

typedef struct CsvRecord {
	char field1[32];
	char field2[32];
	int value;
} CsvRecord;

int sd_read_csv(const char *filename, CsvRecord *records, int max_records, int *record_count) {
	FIL file;
	char line[128];
	*record_count = 0;

	FRESULT res = f_open(&file, filename, FA_READ);
	if (res != FR_OK) {
		printf("Failed to open CSV: %s (%d)", filename, res);
		return res;
	}

	printf("📄 Reading CSV: %s\r\n", filename);
	while (f_gets(line, sizeof(line), &file) && *record_count < max_records) {
		char *token = strtok(line, ",");
		if (!token) continue;
		strncpy(records[*record_count].field1, token, sizeof(records[*record_count].field1));

		token = strtok(NULL, ",");
		if (!token) continue;
		strncpy(records[*record_count].field2, token, sizeof(records[*record_count].field2));

		token = strtok(NULL, ",");
		if (token)
			records[*record_count].value = atoi(token);
		else
			records[*record_count].value = 0;

		(*record_count)++;
	}

	f_close(&file);

	// Print parsed data
	for (int i = 0; i < *record_count; i++) {
		printf("[%d] %s | %s | %d", i,
				records[i].field1,
				records[i].field2,
				records[i].value);
	}

	return FR_OK;
}

int sd_delete_file(const char *filename) {
	FRESULT res = f_unlink(filename);
	printf("Delete %s: %s\r\n", filename, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

int sd_rename_file(const char *oldname, const char *newname) {
	FRESULT res = f_rename(oldname, newname);
	printf("Rename %s to %s: %s\r\n", oldname, newname, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

FRESULT sd_create_directory(const char *path) {
	FRESULT res = f_mkdir(path);
	printf("Create directory %s: %s\r\n", path, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

void sd_list_directory_recursive(const char *path, int depth) {
	DIR dir;
	FILINFO fno;
	//char lfn[256];
	//fno.fname = lfn;
	//fno.lfsize = sizeof(lfn);
	FRESULT res = f_opendir(&dir, path);
	if (res != FR_OK) {
		printf("%*s[ERR] Cannot open: %s\r\n", depth * 2, "", path);
		return;
	}

	while (1) {
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0) break;

		//const char *name = (*fno.fname) ? fno.fname : fno.fname;

		if (fno.fattrib & AM_DIR) {
			if (strcmp(fno.fname, ".") && strcmp(fno.fname, "..")) {
				printf("%*s📁 %s\r\n", depth * 2, "", fno.fname);
				char newpath[128];
				snprintf(newpath, sizeof(newpath), "%s/%s", path, fno.fname);
				sd_list_directory_recursive(newpath, depth + 1);
			}
		} else {
			printf("%*s📄 %s (%lu bytes)\r\n", depth * 2, "", fno.fname, (unsigned long)fno.fsize);
		}
	}
	f_closedir(&dir);
}

void sd_list_files(void) {
	printf("📂 Files on SD Card:\r\n");
	sd_list_directory_recursive(sd_path, 0);
	printf("\r\n\r\n");
}
