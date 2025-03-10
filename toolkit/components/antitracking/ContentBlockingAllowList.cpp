/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "AntiTrackingLog.h"
#include "ContentBlockingAllowList.h"

#include "mozilla/dom/BrowsingContext.h"
#include "mozilla/dom/Document.h"
#include "mozilla/BasePrincipal.h"
#include "mozilla/PermissionManager.h"
#include "mozilla/ScopeExit.h"
#include "nsContentUtils.h"
#include "nsGlobalWindowInner.h"
#include "nsICookieJarSettings.h"
#include "nsIHttpChannel.h"
#include "nsIHttpChannelInternal.h"
#include "nsIPermission.h"
#include "nsNetUtil.h"
#include "nsString.h"

using namespace mozilla;

NS_IMPL_ISUPPORTS(ContentBlockingAllowList, nsIContentBlockingAllowList)

NS_IMETHODIMP
// Wrapper for the static ContentBlockingAllowList::ComputePrincipal method
ContentBlockingAllowList::ComputeContentBlockingAllowListPrincipal(
    nsIPrincipal* aDocumentPrincipal, nsIPrincipal** aPrincipal) {
  NS_ENSURE_ARG_POINTER(aDocumentPrincipal);
  NS_ENSURE_ARG_POINTER(aPrincipal);

  nsCOMPtr<nsIPrincipal> principal;
  ContentBlockingAllowList::ComputePrincipal(aDocumentPrincipal,
                                             getter_AddRefs(principal));

  NS_ENSURE_TRUE(principal, NS_ERROR_FAILURE);

  principal.forget(aPrincipal);

  return NS_OK;
}

/* static */ bool ContentBlockingAllowList::Check(
    nsIPrincipal* aTopWinPrincipal, bool aIsPrivateBrowsing) {
  bool isAllowed = false;
  nsresult rv = Check(aTopWinPrincipal, aIsPrivateBrowsing, isAllowed);
  if (NS_SUCCEEDED(rv) && isAllowed) {
    LOG(
        ("The top-level window is on the content blocking allow list, "
         "bail out early"));
    return true;
  }
  if (NS_FAILED(rv)) {
    LOG(("Checking the content blocking allow list for failed with %" PRIx32,
         static_cast<uint32_t>(rv)));
  }
  return false;
}

/* static */ bool ContentBlockingAllowList::Check(
    nsICookieJarSettings* aCookieJarSettings) {
  if (!aCookieJarSettings) {
    LOG(
        ("Could not check the content blocking allow list because the cookie "
         "jar settings wasn't available"));
    return false;
  }

  return aCookieJarSettings->GetIsOnContentBlockingAllowList();
}

/* static */ bool ContentBlockingAllowList::Check(nsPIDOMWindowInner* aWindow) {
  // TODO: this is a quick fix to ensure that we allow storage permission for
  // a chrome window. We should check if there is a better way to do this in
  // Bug 1626223.
  if (nsGlobalWindowInner::Cast(aWindow)->GetPrincipal() ==
      nsContentUtils::GetSystemPrincipal()) {
    return true;
  }

  // We can check the IsOnContentBlockingAllowList flag in the document's
  // CookieJarSettings. Because this flag represents the fact that whether the
  // top-level document is on the content blocking allow list. And this flag was
  // propagated from the top-level as the CookieJarSettings inherits from the
  // parent.
  RefPtr<dom::Document> doc = nsGlobalWindowInner::Cast(aWindow)->GetDocument();

  if (!doc) {
    LOG(
        ("Could not check the content blocking allow list because the document "
         "wasn't available"));
    return false;
  }

  nsCOMPtr<nsICookieJarSettings> cookieJarSettings = doc->CookieJarSettings();

  return ContentBlockingAllowList::Check(cookieJarSettings);
}

/* static */ bool ContentBlockingAllowList::Check(nsIHttpChannel* aChannel) {
  nsCOMPtr<nsILoadInfo> loadInfo = aChannel->LoadInfo();
  nsCOMPtr<nsICookieJarSettings> cookieJarSettings;

  Unused << loadInfo->GetCookieJarSettings(getter_AddRefs(cookieJarSettings));

  return ContentBlockingAllowList::Check(cookieJarSettings);
}

nsresult ContentBlockingAllowList::Check(
    nsIPrincipal* aContentBlockingAllowListPrincipal, bool aIsPrivateBrowsing,
    bool& aIsAllowListed) {
  MOZ_ASSERT(XRE_IsParentProcess());
  aIsAllowListed = false;

  if (!aContentBlockingAllowListPrincipal) {
    // Nothing to do!
    return NS_OK;
  }

  LOG_PRIN(("Deciding whether the user has overridden content blocking for %s",
            _spec),
           aContentBlockingAllowListPrincipal);

  RefPtr<PermissionManager> permManager = PermissionManager::GetInstance();
  NS_ENSURE_TRUE(permManager, NS_ERROR_FAILURE);

  // Check both the normal mode and private browsing mode user override
  // permissions.
  std::pair<const nsLiteralCString, bool> types[] = {
      {"trackingprotection"_ns, false}, {"trackingprotection-pb"_ns, true}};

  for (const auto& type : types) {
    if (aIsPrivateBrowsing != type.second) {
      continue;
    }

    uint32_t permissions = nsIPermissionManager::UNKNOWN_ACTION;
    nsresult rv = permManager->TestPermissionFromPrincipal(
        aContentBlockingAllowListPrincipal, type.first, &permissions);
    NS_ENSURE_SUCCESS(rv, rv);

    if (permissions == nsIPermissionManager::ALLOW_ACTION) {
      aIsAllowListed = true;
      LOG(("Found user override type %s", type.first.get()));
      // Stop checking the next permisson type if we decided to override.
      break;
    }
  }

  if (!aIsAllowListed) {
    LOG(("No user override found"));
  }

  return NS_OK;
}

/* static */ void ContentBlockingAllowList::ComputePrincipal(
    nsIPrincipal* aDocumentPrincipal, nsIPrincipal** aPrincipal) {
  MOZ_ASSERT(aPrincipal);

  auto returnInputArgument =
      MakeScopeExit([&] { NS_IF_ADDREF(*aPrincipal = aDocumentPrincipal); });

  BasePrincipal* bp = BasePrincipal::Cast(aDocumentPrincipal);
  if (!bp || !bp->IsContentPrincipal()) {
    // If we have something other than a content principal, just return what we
    // have.  This includes the case where we were passed a nullptr.
    return;
  }

  if (aDocumentPrincipal->SchemeIs("chrome") ||
      aDocumentPrincipal->SchemeIs("about")) {
    returnInputArgument.release();
    *aPrincipal = nullptr;
    return;
  }

  // Take the host/port portion so we can allowlist by site. Also ignore the
  // scheme, since users who put sites on the allowlist probably don't expect
  // allowlisting to depend on scheme.
  nsAutoCString escaped("https://"_ns);
  nsAutoCString temp;
  nsresult rv = aDocumentPrincipal->GetHostPort(temp);
  // view-source URIs will be handled by the next block.
  if (NS_FAILED(rv) && !aDocumentPrincipal->SchemeIs("view-source")) {
    // Normal for some loads, no need to print a warning
    return;
  }

  // GetHostPort returns an empty string (with a success error code) for file://
  // URIs.
  if (temp.IsEmpty()) {
    // In this case we want to make sure that our allow list principal would be
    // computed as null.
    returnInputArgument.release();
    *aPrincipal = nullptr;
    return;
  }
  escaped.Append(temp);
  nsCOMPtr<nsIURI> uri;
  rv = NS_NewURI(getter_AddRefs(uri), escaped);
  if (NS_WARN_IF(NS_FAILED(rv))) {
    return;
  }

  nsCOMPtr<nsIPrincipal> principal = BasePrincipal::CreateContentPrincipal(
      uri, aDocumentPrincipal->OriginAttributesRef());
  if (NS_WARN_IF(!principal)) {
    return;
  }

  returnInputArgument.release();
  principal.forget(aPrincipal);
}

/* static */ void ContentBlockingAllowList::RecomputePrincipal(
    nsIURI* aURIBeingLoaded, const OriginAttributes& aAttrs,
    nsIPrincipal** aPrincipal) {
  MOZ_ASSERT(aPrincipal);

  auto returnInputArgument = MakeScopeExit([&] { *aPrincipal = nullptr; });

  // Take the host/port portion so we can allowlist by site. Also ignore the
  // scheme, since users who put sites on the allowlist probably don't expect
  // allowlisting to depend on scheme.
  nsAutoCString escaped("https://"_ns);
  nsAutoCString temp;
  nsresult rv = aURIBeingLoaded->GetHostPort(temp);
  // view-source URIs will be handled by the next block.
  if (NS_FAILED(rv) && !aURIBeingLoaded->SchemeIs("view-source")) {
    // Normal for some loads, no need to print a warning
    return;
  }

  // GetHostPort returns an empty string (with a success error code) for file://
  // URIs.
  if (temp.IsEmpty()) {
    return;
  }
  escaped.Append(temp);

  nsCOMPtr<nsIURI> uri;
  rv = NS_NewURI(getter_AddRefs(uri), escaped);
  if (NS_WARN_IF(NS_FAILED(rv))) {
    return;
  }

  nsCOMPtr<nsIPrincipal> principal =
      BasePrincipal::CreateContentPrincipal(uri, aAttrs);
  if (NS_WARN_IF(!principal)) {
    return;
  }

  returnInputArgument.release();
  principal.forget(aPrincipal);
}

// ContentBlockingAllowListCache

nsresult ContentBlockingAllowListCache::CheckForBaseDomain(
    const nsACString& aBaseDomain, const OriginAttributes& aOriginAttributes,
    bool& aIsAllowListed) {
  MOZ_ASSERT(XRE_IsParentProcess());
  NS_ENSURE_TRUE(!aBaseDomain.IsEmpty(), NS_ERROR_INVALID_ARG);
  aIsAllowListed = false;

  // Ensure we have the permission list.
  nsresult rv = EnsureInit();
  NS_ENSURE_SUCCESS(rv, rv);

  if (aOriginAttributes.IsPrivateBrowsing()) {
    aIsAllowListed = mEntriesPrivateBrowsing.Contains(aBaseDomain);
  } else {
    aIsAllowListed = mEntries.Contains(aBaseDomain);
  }

  return NS_OK;
}

nsTArray<nsCString>
ContentBlockingAllowListCache::GetAllowListPermissionTypes() {
  nsTArray<nsCString> types;
  types.AppendElement("trackingprotection");
  types.AppendElement("trackingprotection-pb");
  return types;
}

nsresult ContentBlockingAllowListCache::IsAllowListPermission(
    nsIPermission* aPermission, bool* aResult) {
  NS_ENSURE_ARG_POINTER(aPermission);
  NS_ENSURE_ARG_POINTER(aResult);

  // Assert that the permission type matches the types returned by
  // GetAllowListPermissionTypes.
#ifdef DEBUG
  nsAutoCString type;
  nsresult rv = aPermission->GetType(type);
  NS_ENSURE_SUCCESS(rv, rv);

  MOZ_ASSERT(type.EqualsLiteral("trackingprotection") ||
             type.EqualsLiteral("trackingprotection-pb"));
#endif

  *aResult = true;
  return NS_OK;
}

nsresult ContentBlockingAllowListCache::EnsureInit() {
  MOZ_ASSERT(XRE_IsParentProcess());

  if (mIsInitialized) {
    return NS_OK;
  }
  mIsInitialized = true;

  // 1. Get all permissions representing allow-list entries.
  RefPtr<PermissionManager> permManager = PermissionManager::GetInstance();
  NS_ENSURE_TRUE(permManager, NS_ERROR_FAILURE);

  nsTArray<nsCString> types = GetAllowListPermissionTypes();

  nsTArray<RefPtr<nsIPermission>> permissions;
  nsresult rv = permManager->GetAllByTypes(types, permissions);
  NS_ENSURE_SUCCESS(rv, rv);

  // 2. Populate mEntries and mEntriesPrivateBrowsing from permission list for
  //    faster lookup.
  for (auto& permission : permissions) {
    MOZ_ASSERT(permission);

    // Check if the permission is a suitable allow-list permission.
    bool result = false;
    nsresult rv = IsAllowListPermission(permission, &result);
    NS_ENSURE_SUCCESS(rv, rv);

    if (!result) {
      continue;
    }

    // Permission is suitable, extract base domain from permission principal.
    nsCOMPtr<nsIPrincipal> principal;
    rv = permission->GetPrincipal(getter_AddRefs(principal));
    NS_ENSURE_SUCCESS(rv, rv);
    MOZ_ASSERT(principal);

    nsAutoCString baseDomain;
    rv = principal->GetBaseDomain(baseDomain);
    NS_ENSURE_SUCCESS(rv, rv);

    // Sort base domains into sets for normal / private browsing.
    if (principal->OriginAttributesRef().IsPrivateBrowsing()) {
      mEntriesPrivateBrowsing.Insert(baseDomain);
    } else {
      mEntries.Insert(baseDomain);
    }
  }

  return NS_OK;
}
