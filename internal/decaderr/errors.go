// Package decaderr declares the sentinel errors of decad's public API so that
// internal packages can return them without importing the root package.
// The root package re-exports each one under the same name; the two names
// hold the same value, so errors.Is matches either. Each sentinel's meaning
// is documented on its root re-export in errors.go.
package decaderr

import "errors"

// ErrNoMatch is the value behind decad.ErrNoMatch.
var ErrNoMatch = errors.New("decad: selector matched nothing")

// ErrCardinality is the value behind decad.ErrCardinality.
var ErrCardinality = errors.New("decad: cardinality assertion failed")

// ErrBodyReportNotFound is the value behind decad.ErrBodyReportNotFound.
var ErrBodyReportNotFound = errors.New("decad: body is not represented in this report")

// ErrForeignBody is the value behind decad.ErrForeignBody.
var ErrForeignBody = errors.New("decad: body is owned by a different document")

// ErrForeignProfile is the value behind decad.ErrForeignProfile.
var ErrForeignProfile = errors.New("decad: profile was built from a different sketch")

// ErrStaleProfile is the value behind decad.ErrStaleProfile.
var ErrStaleProfile = errors.New("decad: profile is stale")

// ErrRetiredBody is the value behind decad.ErrRetiredBody.
var ErrRetiredBody = errors.New("decad: body has been retired from its document")

// ErrNegativeMagnitude is the value behind decad.ErrNegativeMagnitude.
var ErrNegativeMagnitude = errors.New("decad: negative magnitude")

// ErrUnrecordableProfile is the value behind decad.ErrUnrecordableProfile.
var ErrUnrecordableProfile = errors.New("decad: profile boundary cannot be recorded exactly")

// ErrNotSolid is the value behind decad.ErrNotSolid.
var ErrNotSolid = errors.New("decad: body is not a solid")

// ErrDegenerate is the value behind decad.ErrDegenerate.
var ErrDegenerate = errors.New("decad: degenerate input")

// ErrBooleanFailed is the value behind decad.ErrBooleanFailed.
var ErrBooleanFailed = errors.New("decad: boolean operation failed")

// ErrInvalidProfile is the value behind decad.ErrInvalidProfile.
var ErrInvalidProfile = errors.New("decad: profile is not a valid region")

// ErrUnitKind is the value behind decad.ErrUnitKind.
var ErrUnitKind = errors.New("decad: wrong unit kind")

// ErrNotFinite is the value behind decad.ErrNotFinite.
var ErrNotFinite = errors.New("decad: non-finite value")

// ErrUnsupported is the value behind decad.ErrUnsupported.
var ErrUnsupported = errors.New("decad: not supported by the current evaluator")
