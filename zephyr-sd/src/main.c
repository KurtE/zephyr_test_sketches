#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(main);

static int lsdir(const char *path);

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

/*
*  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
*  in ffconf.h
*/
static const char *disk_mount_pt = "/SD:";

static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}

static int mount_sd_card(void)
{
	/* raw disk i/o */
	static const char *disk_pdrv = "SD";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	if (disk_access_init(disk_pdrv) != 0) {
		LOG_ERR("Storage init ERROR!");
		return -1;
	}

	if (disk_access_ioctl(disk_pdrv,
			DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
		LOG_ERR("Unable to get sector count");
		return -1;
	}
	LOG_INF("Block count %u", block_count);

	if (disk_access_ioctl(disk_pdrv,
			DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
		LOG_ERR("Unable to get sector size");
		return -1;
	}
	printk("Sector size %u\n", block_size);

	memory_size_mb = (uint64_t)block_count * block_size;
	printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		printk("Disk mounted.\n");
		lsdir(disk_mount_pt);
	} else {
		printk("Failed to mount disk - trying one more time\n");
		res = fs_mount(&mp);
		if (res != FR_OK) {
			printk("Error mounting disk.\n");
			return -1;
		}
	}

	return 0;
}

int main(void)
{
	int ret = 0;
	/** SD card mount test **/
	if (mount_sd_card()) {
		printk("Failed to mount SD card\n");
		return -1;
	} else {
		printk("Successfully mounted SD card\n");
	}

	char file_data_buffer[200];
	struct fs_file_t data_filp;
	fs_file_t_init(&data_filp);

	char file_ch;
	ret = fs_unlink("/SD:/test_data.txt");

	ret = fs_open(&data_filp, "/SD:/test_data.txt", FS_O_WRITE | FS_O_CREATE);
	if (ret) {
		printk("%s -- failed to create file (err = %d)\n", __func__, ret);
		return -2;
	} else {
		printk("%s - successfully created file\n", __func__);
	}

	sprintf(file_data_buffer, "hello world!\n");
	ret = fs_write(&data_filp, file_data_buffer, strlen(file_data_buffer));
	fs_close(&data_filp);

	while (1) {
		k_sleep(K_MSEC(1));
	}

	return 0;
}
