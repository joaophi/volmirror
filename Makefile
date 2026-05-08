# VolMirror — CoreAudio HAL plugin + userland agent
#
# Build:           make
# Install:         sudo make install
# Uninstall:       sudo make uninstall
# Reload all:      sudo make reload
# Reload agent:    sudo make reload-agent     (no coreaudiod kill)
# Reload driver:   sudo make reload-driver

BUNDLE        := VolMirror
EXT           := driver
BUILD_DIR     := build
BUNDLE_DIR    := $(BUILD_DIR)/$(BUNDLE).$(EXT)
CONTENTS_DIR  := $(BUNDLE_DIR)/Contents
MACOS_DIR     := $(CONTENTS_DIR)/MacOS
EXECUTABLE    := $(MACOS_DIR)/$(BUNDLE)
INSTALL_DIR   := /Library/Audio/Plug-Ins/HAL

AGENT_BIN            := $(BUILD_DIR)/VolMirrorAgent
AGENT_INSTALL_DIR    := /usr/local/libexec
AGENT_INSTALL        := $(AGENT_INSTALL_DIR)/VolMirrorAgent
AGENT_PLIST          := com.joaopedro.VolMirror.agent.plist
AGENT_PLIST_INSTALL  := /Library/LaunchAgents/$(AGENT_PLIST)
AGENT_LABEL          := com.joaopedro.VolMirror.agent

# Resolve the human user's UID even when running under sudo.
USER_UID             := $(shell id -u $${SUDO_USER:-$$USER})

CC             := clang
CFLAGS         := -O2 -Wall -Wextra -fvisibility=hidden -mmacosx-version-min=12.0
LDFLAGS_BUNDLE := -bundle -framework CoreAudio -framework CoreFoundation
LDFLAGS_AGENT  := -framework CoreAudio -framework CoreFoundation

.PHONY: all clean install uninstall reload reload-driver reload-agent

all: $(EXECUTABLE) $(CONTENTS_DIR)/Info.plist $(AGENT_BIN)
	@codesign --force --sign - $(BUNDLE_DIR)
	@echo "Built $(BUNDLE_DIR) and $(AGENT_BIN)"

$(EXECUTABLE): Source/Plugin.c | $(MACOS_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS_BUNDLE) -o $@ $<

$(AGENT_BIN): Source/Agent.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS_AGENT) -o $@ $<

$(CONTENTS_DIR)/Info.plist: Source/Info.plist | $(CONTENTS_DIR)
	cp $< $@

$(MACOS_DIR) $(CONTENTS_DIR) $(BUILD_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

install: all
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	cp -R $(BUNDLE_DIR) $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	mkdir -p $(AGENT_INSTALL_DIR)
	install -m 755 $(AGENT_BIN) $(AGENT_INSTALL)
	install -m 644 -o root -g wheel Source/$(AGENT_PLIST) $(AGENT_PLIST_INSTALL)
	# Strip quarantine + warm Gatekeeper so the malware scan happens here
	# instead of stalling the next login (avoids a multi-second hold on
	# the user-session launchd queue at boot).
	-@xattr -dr com.apple.quarantine $(INSTALL_DIR)/$(BUNDLE).$(EXT) $(AGENT_INSTALL) 2>/dev/null || true
	-@spctl --assess --type execute $(AGENT_INSTALL) >/dev/null 2>&1 || true
	$(MAKE) reload

uninstall:
	-@launchctl bootout gui/$(USER_UID)/$(AGENT_LABEL) 2>/dev/null || true
	-@killall -9 VolMirrorAgent 2>/dev/null || true
	rm -f $(AGENT_INSTALL) $(AGENT_PLIST_INSTALL)
	rm -rf $(INSTALL_DIR)/$(BUNDLE).$(EXT)
	$(MAKE) reload-driver

reload: reload-driver reload-agent

reload-driver:
	@killall coreaudiod

# If the agent's already bootstrapped, kickstart it (instant restart, works on
# user-domain services even with SIP). Only bootstrap on first install.
reload-agent:
	@if launchctl print gui/$(USER_UID)/$(AGENT_LABEL) >/dev/null 2>&1; then \
		launchctl kickstart -k gui/$(USER_UID)/$(AGENT_LABEL); \
	else \
		launchctl bootstrap gui/$(USER_UID) $(AGENT_PLIST_INSTALL); \
	fi
