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
HELPER        := com.apple.audio.Core-Audio-Driver-Service.helper

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
	# Atomic bundle swap: stage alongside the live bundle, then mv into
	# place — there's no in-between state where the directory exists but
	# is incomplete, so launchd / coreaudiod can't load a half-copied bundle.
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT).new
	cp -R $(BUNDLE_DIR) $(INSTALL_DIR)/$(BUNDLE).$(EXT).new
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	mv $(INSTALL_DIR)/$(BUNDLE).$(EXT).new $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	$(MAKE) reload
	@echo
	@echo "Installed. If macOS prompts about a system extension, approve in"
	@echo "System Settings > Privacy & Security, then run: sudo make reload"

uninstall:
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	$(MAKE) reload

reload:
	# SIGTERM first so coreaudiod can cleanly disconnect from its clients
	# (Control Center, audio apps, etc) — pure SIGKILL leaves them stuck
	# discovering the dead connection by timeout, which manifests as a
	# multi-minute UI sluggishness afterwards.
	# Bounded wait (~3 s); SIGKILL anything that's still alive after that
	# so we don't pay the helper's own multi-minute graceful-shutdown
	# grace period (it holds a `start_io` os_transaction open until it
	# times out otherwise).
	@killall coreaudiod 2>/dev/null || true
	@killall $(HELPER) 2>/dev/null || true
	@for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do \
		pgrep -x coreaudiod >/dev/null || pgrep -x $(HELPER) >/dev/null || break; \
		sleep 0.2; \
	done
	@killall -9 coreaudiod 2>/dev/null || true
	@killall -9 $(HELPER) 2>/dev/null || true
