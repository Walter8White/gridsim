// Read/write selected LJ-S8000 program settings from Linux.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LJS8_ErrorCode.h"
#include "LJS8_IF_Linux.h"

struct SettingDef {
	const char* name;
	unsigned char category;
	unsigned char item;
	unsigned int size;
	int minValue;
	int maxValue;
	const char* help;
};

static const SettingDef kSettings[] = {
	{"exposure", 0x04, 0x00, 4, 0, 15, "0:15us 1:30us 2:60us 3:80us 4:120us 5:160us 6:210us 7:240us 8:320us 9:380us 10:480us 11:640us 12:960us 13:1700us 14:4800us 15:9600us"},
	{"dynamic_range", 0x04, 0x01, 4, 1, 9, "CMOS dynamic range, 1..9"},
	{"light_mode", 0x04, 0x02, 4, 0, 2, "0:MANUAL 1:AUTO 2:SLOPE"},
	{"light_upper", 0x04, 0x03, 4, 1, 99, "Light intensity control upper limit, 1..99"},
	{"light_lower", 0x04, 0x04, 4, 1, 99, "Light intensity control lower limit, 1..99"},
	{"x_subsample", 0x03, 0x00, 4, 1, 2, "Points to sub-sample on X axis, 1..2"},
	{"y_subsample", 0x03, 0x01, 4, 1, 8, "Points to sub-sample on Y axis, 1..8"},
	{"detection_sensitivity", 0x05, 0x00, 4, 1, 5, "Peak detection sensitivity, 1 low .. 5 high"},
	{"dead_zone_interpolation", 0x06, 0x00, 4, 0, 2, "0:off 1:horizontal_vertical 2:linear"},
};

static const SettingDef* find_setting(const char* name) {
	for (const auto& setting : kSettings) {
		if (strcmp(setting.name, name) == 0)
			return &setting;
	}
	return NULL;
}

static void usage(const char* argv0) {
	printf("usage:\n");
	printf("  %s list\n", argv0);
	printf("  %s get [--program N] all|<setting>|peak_width_filter\n", argv0);
	printf("  %s set [--program N] [--save] <setting> <value>\n", argv0);
	printf("  %s set [--program N] [--save] peak_width_filter off|on [strength]\n", argv0);
	printf("\nDefault program is the active program. Default write target is RUNNING only.\n");
	printf("Use --save only when you want the setting to survive power-off.\n");
}

static int open_device(int deviceId) {
	LJS8IF_ETHERNET_CONFIG ethernetConfig = {
		{192, 168, 0, 1},
		24691
	};
	return LJS8IF_EthernetOpen(deviceId, &ethernetConfig);
}

static LJS8IF_TARGET_SETTING target_for_program(int program, unsigned char category, unsigned char item) {
	LJS8IF_TARGET_SETTING target = {};
	target.byType = static_cast<unsigned char>(0x10 + program);
	target.byCategory = category;
	target.byItem = item;
	return target;
}

static int get_u8_setting(int deviceId, int program, const SettingDef& setting, unsigned char* value) {
	unsigned char data[4] = {};
	LJS8IF_TARGET_SETTING target = target_for_program(program, setting.category, setting.item);
	int rc = LJS8IF_GetSetting(deviceId, LJS8IF_SETTING_DEPTH_RUNNING, target, data, setting.size);
	if (rc == LJS8IF_RC_OK)
		*value = data[0];
	return rc;
}

static int set_u8_setting(int deviceId, int program, const SettingDef& setting, int value, unsigned char depth) {
	if (value < setting.minValue || value > setting.maxValue) {
		printf("%s out of range: %d (%s)\n", setting.name, value, setting.help);
		return 2;
	}

	unsigned char data[4] = {static_cast<unsigned char>(value), 0, 0, 0};
	unsigned int settingError = 0;
	LJS8IF_TARGET_SETTING target = target_for_program(program, setting.category, setting.item);
	int rc = LJS8IF_SetSetting(deviceId, depth, target, data, setting.size, &settingError);
	printf("SetSetting %s=%d depth=%u rc=0x%x setting_error=0x%x\n",
		setting.name, value, depth, rc, settingError);
	return rc == LJS8IF_RC_OK ? 0 : 1;
}

static int get_peak_width_filter(int deviceId, int program) {
	unsigned char data[4] = {};
	LJS8IF_TARGET_SETTING target = target_for_program(program, 0x05, 0x03);
	int rc = LJS8IF_GetSetting(deviceId, LJS8IF_SETTING_DEPTH_RUNNING, target, data, sizeof(data));
	printf("peak_width_filter rc=0x%x enabled=%u strength=%u raw=%02x %02x %02x %02x\n",
		rc, data[0], data[1], data[0], data[1], data[2], data[3]);
	return rc == LJS8IF_RC_OK ? 0 : 1;
}

static int set_peak_width_filter(int deviceId, int program, const char* enabledText, int strength, unsigned char depth) {
	int enabled;
	if (strcmp(enabledText, "on") == 0 || strcmp(enabledText, "1") == 0)
		enabled = 1;
	else if (strcmp(enabledText, "off") == 0 || strcmp(enabledText, "0") == 0)
		enabled = 0;
	else {
		printf("peak_width_filter expects off|on [strength]\n");
		return 2;
	}

	if (strength < 1 || strength > 5) {
		printf("peak_width_filter strength out of range: %d (1..5)\n", strength);
		return 2;
	}

	unsigned char data[4] = {static_cast<unsigned char>(enabled), static_cast<unsigned char>(strength), 0, 0};
	unsigned int settingError = 0;
	LJS8IF_TARGET_SETTING target = target_for_program(program, 0x05, 0x03);
	int rc = LJS8IF_SetSetting(deviceId, depth, target, data, sizeof(data), &settingError);
	printf("SetSetting peak_width_filter enabled=%d strength=%d depth=%u rc=0x%x setting_error=0x%x\n",
		enabled, strength, depth, rc, settingError);
	return rc == LJS8IF_RC_OK ? 0 : 1;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		usage(argv[0]);
		return 2;
	}

	if (strcmp(argv[1], "list") == 0) {
		for (const auto& setting : kSettings)
			printf("%s: %s\n", setting.name, setting.help);
		printf("peak_width_filter: off|on strength 1..5\n");
		return 0;
	}

	int program = -1;
	unsigned char depth = LJS8IF_SETTING_DEPTH_RUNNING;
	int arg = 2;
	while (arg < argc) {
		if (strcmp(argv[arg], "--program") == 0 && arg + 1 < argc) {
			program = atoi(argv[arg + 1]);
			arg += 2;
		} else if (strcmp(argv[arg], "--save") == 0) {
			depth = LJS8IF_SETTING_DEPTH_SAVE;
			arg += 1;
		} else {
			break;
		}
	}

	if (strcmp(argv[1], "get") != 0 && strcmp(argv[1], "set") != 0) {
		usage(argv[0]);
		return 2;
	}

	int deviceId = 0;
	int rc = open_device(deviceId);
	if (rc != LJS8IF_RC_OK) {
		printf("EthernetOpen rc=0x%x\n", rc);
		return 1;
	}

	if (program < 0) {
		unsigned char activeProgram = 0;
		rc = LJS8IF_GetActiveProgram(deviceId, &activeProgram);
		if (rc != LJS8IF_RC_OK) {
			printf("GetActiveProgram rc=0x%x\n", rc);
			LJS8IF_CommunicationClose(deviceId);
			return 1;
		}
		program = activeProgram;
	}

	if (program < 0 || program > 15) {
		printf("Program must be 0..15, got %d\n", program);
		LJS8IF_CommunicationClose(deviceId);
		return 2;
	}
	printf("program=%d\n", program);

	int exitCode = 0;
	if (strcmp(argv[1], "get") == 0) {
		if (arg >= argc) {
			usage(argv[0]);
			exitCode = 2;
		} else if (strcmp(argv[arg], "all") == 0) {
			for (const auto& setting : kSettings) {
				unsigned char value = 0;
				rc = get_u8_setting(deviceId, program, setting, &value);
				printf("%s rc=0x%x value=%u (%s)\n", setting.name, rc, value, setting.help);
				if (rc != LJS8IF_RC_OK)
					exitCode = 1;
			}
			if (get_peak_width_filter(deviceId, program) != 0)
				exitCode = 1;
		} else if (strcmp(argv[arg], "peak_width_filter") == 0) {
			exitCode = get_peak_width_filter(deviceId, program);
		} else {
			const SettingDef* setting = find_setting(argv[arg]);
			if (setting == NULL) {
				printf("unknown setting: %s\n", argv[arg]);
				exitCode = 2;
			} else {
				unsigned char value = 0;
				rc = get_u8_setting(deviceId, program, *setting, &value);
				printf("%s rc=0x%x value=%u (%s)\n", setting->name, rc, value, setting->help);
				exitCode = rc == LJS8IF_RC_OK ? 0 : 1;
			}
		}
	} else {
		if (arg + 1 >= argc) {
			usage(argv[0]);
			exitCode = 2;
		} else if (strcmp(argv[arg], "peak_width_filter") == 0) {
			int strength = arg + 2 < argc ? atoi(argv[arg + 2]) : 1;
			exitCode = set_peak_width_filter(deviceId, program, argv[arg + 1], strength, depth);
		} else {
			const SettingDef* setting = find_setting(argv[arg]);
			if (setting == NULL) {
				printf("unknown setting: %s\n", argv[arg]);
				exitCode = 2;
			} else {
				exitCode = set_u8_setting(deviceId, program, *setting, atoi(argv[arg + 1]), depth);
			}
		}
	}

	LJS8IF_CommunicationClose(deviceId);
	return exitCode;
}
