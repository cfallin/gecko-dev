// |reftest| shell-option(--enable-temporal) skip-if(!this.hasOwnProperty('Temporal')||!xulRuntime.shell) -- Temporal is not enabled unconditionally, requires shell-options
// Copyright (C) 2022 Igalia, S.L. All rights reserved.
// This code is governed by the BSD license found in the LICENSE file.

/*---
esid: sec-temporal.plainyearmonth.prototype.until
description: Verify that invalid options are handled correctly.
features: [Temporal]
---*/

const feb20 = new Temporal.PlainYearMonth(2020, 2);
const feb21 = new Temporal.PlainYearMonth(2021, 2);

[
  null,
  1,
  "hello",
  true,
  Symbol("foo"),
  1n
].forEach((badOption) =>
  assert.throws(TypeError, () => feb20.until(feb21, badOption), `${String(badOption)} throws TypeError`)
);

reportCompare(0, 0);
