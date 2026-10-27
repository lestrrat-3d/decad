package freeform

// SpanLengthBracketUncached is the bracket without the cross-document cache,
// so benchmarks and differential tests measure the arithmetic itself on every
// call.
var SpanLengthBracketUncached = spanLengthBracket

// ShiftedCmp is the bracket's exact a·2^s against b comparison on
// normalised magnitudes.
var ShiftedCmp = shiftedCmp
