# VolMirror — CoreAudio HAL plugin
#
# Build:     make
# Install:   sudo make install
# Uninstall: sudo make uninstall
# Reload:    sudo make reload   (restart coreaudiod without re-copying)

BUNDLE        := VolMirror
EXT           := driver
BUILD_DIR     := build
BUNDLE_DIR    := $(BUILD_DIR)/$(BUNDLE).$(EXT)
CONTENTS_DIR  := $(BUNDLE_DIR)/Contents
MACOS_DIR     := $(CONTENTS_DIR)/MacOS
EXECUTABLE    := $(MACOS_DIR)/$(BUNDLE)
INSTALL_DIR   := /Library/Audio/Plug-Ins/HAL

CC            := clang
CFLAGS        := -O2 -Wall -Wextra -fvisibility=hidden -mmacosx-version-min=12.0
LDFLAGS       := -bundle -framework CoreAudio -framework CoreFoundation

.PHONY: all clean install uninstall reload

all: $(EXECUTABLE) $(CONTENTS_DIR)/Info.plist
	@codesign --force --sign - $(BUNDLE_DIR)
	@echo "Built $(BUNDLE_DIR)"

$(EXECUTABLE): Source/Plugin.c | $(MACOS_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

$(CONTENTS_DIR)/Info.plist: Source/Info.plist | $(CONTENTS_DIR)
	cp $< $@

$(MACOS_DIR) $(CONTENTS_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

install: all
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	cp -R $(BUNDLE_DIR) $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	$(MAKE) reload

uninstall:
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	$(MAKE) reload

reload:
	@killall coreaudiod
