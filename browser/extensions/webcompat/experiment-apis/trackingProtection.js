/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

/* global ExtensionAPI, ExtensionCommon, ExtensionParent, Services, XPCOMUtils */

// eslint-disable-next-line mozilla/reject-importGlobalProperties
XPCOMUtils.defineLazyGlobalGetters(this, ["URL", "ChannelWrapper"]);

class AllowList {
  constructor(id) {
    this._id = id;
  }

  setShims(patterns, notHosts) {
    this._shimPatterns = patterns;
    this._shimMatcher = new MatchPatternSet(patterns || []);
    this._shimNotHosts = notHosts || [];
    return this;
  }

  setAllows(patterns, hosts) {
    this._allowPatterns = patterns;
    this._allowMatcher = new MatchPatternSet(patterns || []);
    this._allowHosts = hosts || [];
    return this;
  }

  shims(url, topHost) {
    return (
      this._shimMatcher?.matches(url) && !this._shimNotHosts?.includes(topHost)
    );
  }

  allows(url, topHost) {
    return (
      this._allowMatcher?.matches(url) && this._allowHosts?.includes(topHost)
    );
  }
}

class Manager {
  constructor() {
    this._allowLists = new Map();
  }

  _getAllowList(id) {
    if (!this._allowLists.has(id)) {
      this._allowLists.set(id, new AllowList(id));
    }
    return this._allowLists.get(id);
  }

  _ensureStarted() {
    if (this._classifierObserver) {
      return;
    }

    this._unblockedChannelIds = new Set();
    this._channelClassifier = Cc[
      "@mozilla.org/url-classifier/channel-classifier-service;1"
    ].getService(Ci.nsIChannelClassifierService);
    this._classifierObserver = {};
    this._classifierObserver.observe = (subject, topic) => {
      switch (topic) {
        case "http-on-stop-request": {
          const { channelId } = subject.QueryInterface(Ci.nsIIdentChannel);
          this._unblockedChannelIds.delete(channelId);
          break;
        }
        case "urlclassifier-before-block-channel": {
          const channel = subject.QueryInterface(
            Ci.nsIUrlClassifierBlockedChannel
          );
          const { channelId, url } = channel;
          let topHost;
          try {
            topHost = new URL(channel.topLevelUrl).hostname;
          } catch (_) {
            return;
          }
          // If anti-tracking webcompat is disabled, we only permit replacing
          // channels, not fully unblocking them.
          if (Manager.ENABLE_WEBCOMPAT) {
            // if any allowlist unblocks the request entirely, we allow it
            for (const allowList of this._allowLists.values()) {
              if (allowList.allows(url, topHost)) {
                this._unblockedChannelIds.add(channelId);
                channel.allow();
                return;
              }
            }
          }
          // otherwise, if any allowlist shims the request we say it's replaced
          for (const allowList of this._allowLists.values()) {
            if (allowList.shims(url, topHost)) {
              this._unblockedChannelIds.add(channelId);
              channel.replace();
              return;
            }
          }
          break;
        }
      }
    };
    Services.obs.addObserver(this._classifierObserver, "http-on-stop-request");
    this._channelClassifier.addListener(this._classifierObserver);
  }

  stop() {
    if (!this._classifierObserver) {
      return;
    }

    Services.obs.removeObserver(
      this._classifierObserver,
      "http-on-stop-request"
    );
    this._channelClassifier.removeListener(this._classifierObserver);
    delete this._channelClassifier;
    delete this._classifierObserver;
  }

  wasChannelIdUnblocked(channelId) {
    return this._unblockedChannelIds?.has(channelId);
  }

  allow(allowListId, patterns, hosts) {
    this._ensureStarted();
    this._getAllowList(allowListId).setAllows(patterns, hosts);
  }

  shim(allowListId, patterns, notHosts) {
    this._ensureStarted();
    this._getAllowList(allowListId).setShims(patterns, notHosts);
  }

  revoke(allowListId) {
    this._allowLists.delete(allowListId);
  }
}
var manager = new Manager();

function getChannelId(context, requestId) {
  const wrapper = ChannelWrapper.getRegisteredChannel(
    requestId,
    context.extension.policy,
    context.xulBrowser.frameLoader.remoteTab
  );
  return wrapper?.channel?.QueryInterface(Ci.nsIIdentChannel)?.channelId;
}

var dFPIPrefName = "network.cookie.cookieBehavior";
var dFPIPbPrefName = "network.cookie.cookieBehavior.pbmode";
var dFPIStatus;
function updateDFPIStatus() {
  dFPIStatus = {
    nonPbMode: 5 == Services.prefs.getIntPref(dFPIPrefName),
    pbMode: 5 == Services.prefs.getIntPref(dFPIPbPrefName),
  };
}

this.trackingProtection = class extends ExtensionAPI {
  onShutdown() {
    if (manager) {
      manager.stop();
    }
    Services.prefs.removeObserver(dFPIPrefName, updateDFPIStatus);
    Services.prefs.removeObserver(dFPIPbPrefName, updateDFPIStatus);
  }

  getAPI(context) {
    const {
      extension: { tabManager },
    } = this;
    const EventManager = ExtensionCommon.EventManager;
    Services.prefs.addObserver(dFPIPrefName, updateDFPIStatus);
    Services.prefs.addObserver(dFPIPbPrefName, updateDFPIStatus);
    updateDFPIStatus();

    return {
      trackingProtection: {
        onSmartBlockEmbedUnblock: new EventManager({
          context,
          name: "trackingProtection.onSmartBlockEmbedUnblock",
          register: fire => {
            const callback = (subject, topic, data) => {
              // chrome tab id needs to be converted to extension tab id
              let hostname = subject.linkedBrowser.currentURI.host;
              let tabId = tabManager.convert(subject).id;
              fire.sync(tabId, data, hostname);
            };
            Services.obs.addObserver(callback, "smartblock:unblock-embed");
            return () => {
              Services.obs.removeObserver(callback, "smartblock:unblock-embed");
            };
          },
        }).api(),
        onSmartBlockEmbedReblock: new EventManager({
          context,
          name: "trackingProtection.onSmartBlockEmbedReblock",
          register: fire => {
            const callback = (subject, topic, data) => {
              // chrome tab id needs to be converted to extension tab id
              let hostname = subject.linkedBrowser.currentURI.host;
              let tabId = tabManager.convert(subject).id;
              fire.sync(tabId, data, hostname);
            };
            Services.obs.addObserver(callback, "smartblock:reblock-embed");
            return () => {
              Services.obs.removeObserver(callback, "smartblock:reblock-embed");
            };
          },
        }).api(),
        async shim(allowListId, patterns, notHosts) {
          manager.shim(allowListId, patterns, notHosts);
        },
        async allow(allowListId, patterns, hosts) {
          manager.allow(allowListId, patterns, hosts);
        },
        async revoke(allowListId) {
          manager.revoke(allowListId);
        },
        async wasRequestUnblocked(requestId) {
          if (!manager) {
            return false;
          }
          const channelId = getChannelId(context, requestId);
          if (!channelId) {
            return false;
          }
          return manager.wasChannelIdUnblocked(channelId);
        },
        async isDFPIActive(isPrivate) {
          if (isPrivate) {
            return dFPIStatus.pbMode;
          }
          return dFPIStatus.nonPbMode;
        },
        openProtectionsPanel(tabId) {
          let tab = tabManager.get(tabId);
          if (!tab.active) {
            // break if tab is not the active tab
            return;
          }

          let win = tab?.window;
          Services.obs.notifyObservers(
            win.gBrowser.selectedBrowser.browsingContext,
            "smartblock:open-protections-panel"
          );
        },
        async getSmartBlockEmbedFluentString(tabId, shimId, websiteHost) {
          let win = tabManager.get(tabId).window;
          let document = win.document;

          let { gProtectionsHandler } = win.gBrowser.ownerGlobal;
          let { displayName } = gProtectionsHandler.smartblockEmbedInfo.find(
            element => element.shimId == shimId
          );

          let fluentArgs = [
            {
              id: "smartblock-placeholder-title",
              args: {
                trackername: displayName,
              },
            },
            {
              id: "smartblock-placeholder-desc",
            },
            {
              id: "smartblock-placeholder-button-text",
              args: { websitehost: websiteHost },
            },
          ];

          return document.l10n.formatValues(fluentArgs);
        },
      },
    };
  }
};

XPCOMUtils.defineLazyPreferenceGetter(
  Manager,
  "ENABLE_WEBCOMPAT",
  "privacy.antitracking.enableWebcompat",
  false
);
