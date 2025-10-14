Required to replace existing SD_MMC files so that SD_MMC.end() releases its LDO power handle
so that SD_MMC.begin() can be called successfully again after SD_MMC.end().

This enables same/different cards to be removed/inserted, mounted successfully without
LDO errors.