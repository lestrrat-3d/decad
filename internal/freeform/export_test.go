package freeform

// SpanLengthBracketUncached is the bracket without the cross-document cache,
// so benchmarks and differential tests measure the arithmetic itself on every
// call.
var SpanLengthBracketUncached = spanLengthBracket
