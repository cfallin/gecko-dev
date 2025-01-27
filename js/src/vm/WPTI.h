/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef vm_WPTI_h
#define vm_WPTI_h

/*
 * Whole-program type inference.
 */

#include "jspubtd.h"

#include "js/GCVector.h"
#include "vm/JSObject.h"

namespace js {
namespace wpti {

bool Run(JSContext* cx, Handle<GCVector<JSObject*>> roots);

} /* namespace wpti */
} /* namespace js */

#endif /* vm_WPTI_h */
