package decad

import (
	"errors"
	"fmt"
)

// The sentinel errors of the public API (docs/api-design.md §12): the cases an
// agent must branch on. Every fallible operation returns (T, error) — never a
// bool, a -1 sentinel, or a deferred health state — and wraps one of these
// where the failure is one a caller decides on, so errors.Is is the branch.

// ErrNoMatch is returned when a selector that asserts no cardinality at all
// resolves to zero entities. A selector that DOES carry a cardinality
// assertion — Exactly(n) / AtLeast(n), or the implicit exactly-one of a
// ToFace, ToFaceAngular or EdgeAxis — reports [ErrCardinality] instead, even
// at zero matches.
var ErrNoMatch = errors.New("decad: selector matched nothing")

// ErrCardinality is returned when a cardinality assertion fails: an explicit
// Exactly(n) / AtLeast(n) on a query, or the implicit exactly-one of a ToFace,
// ToFaceAngular or EdgeAxis. It takes precedence at zero matches — a failed
// assertion is ErrCardinality even when the selector matched nothing.
var ErrCardinality = errors.New("decad: cardinality assertion failed")

// ErrForeignBody is returned when an operation is handed bodies owned by
// different documents, or when an extent or axis names a body owned by a
// document other than the one the feature is called on. Bodies from different
// documents cannot be combined: the result would have no owner, and a face in
// another document's coordinates would stop a sweep somewhere the caller
// never named.
var ErrForeignBody = errors.New("decad: body is owned by a different document")

// ErrForeignProfile is returned when a feature is handed a profile built from
// a different sketch than the one given, or when the profile's mutable boundary
// contains an entity the source sketch does not own. A foreign profile or
// boundary is expressed in another sketch's plane-local coordinates, so
// lifting it through the given sketch's frame would silently misplace it.
var ErrForeignProfile = errors.New("decad: profile was built from a different sketch")

// ErrStaleProfile is returned when a feature is handed a profile built before
// the sketch's current state — Profile.IsStale(), sketch's own staleness
// answer. A stale profile's boundary is the old geometry; sweeping it would
// silently build the wrong part. The caller rebuilds with s.Profiles() and
// passes a current one.
var ErrStaleProfile = errors.New("decad: profile is stale")

// ErrRetiredBody is returned when an operation, or an extent or axis, is
// handed a body its document has retired. A retired body remains readable,
// but it is no longer part of the model: no operation takes one.
var ErrRetiredBody = errors.New("decad: body has been retired from its document")

// ErrUnresolvedBody is returned when a StepRef is passed as a BodyRef to a
// feature call, where a live body is required. A StepRef names a step of a
// recorded Recipe; a feature call resolves selectors against live geometry.
var ErrUnresolvedBody = errors.New("decad: a live body is required, not a step reference")

// ErrNegativeMagnitude is returned when a magnitude is given as a negative
// value. Magnitudes — extent distances and angles, fillet and chamfer sizes,
// shell thicknesses, predicate lengths, tolerances and tool sizes — are
// non-negative; sense is carried by an enumerated Direction, never by a sign.
// The two signed displacements, ToFace.Offset and the extrude taper, are
// outside it: neither carries a Direction to reverse, so a negative value
// there is a legal intent.
var ErrNegativeMagnitude = errors.New("decad: negative magnitude")

// ErrUnrecordableProfile is returned when a feature is handed a valid profile
// whose boundary decad cannot record exactly: a Partial fragment whose cut
// sketch reports sampled (BoundaryEdge.TExact == false), or one whose
// certified range the seam's one-sided falsifier disproves. A Step that
// recorded the whole curve where the caller drew a piece of it, or an
// approximate range as an exact trim, would be a lossy record of intent, so
// decad rejects — it never repairs, projects, fits or solves for a point.
// Full semantics in docs/sketch-seam-design.md.
var ErrUnrecordableProfile = errors.New("decad: profile boundary cannot be recorded exactly")

// ErrNotSolid is returned by a region quantity of a body that is not a valid
// solid — a volume or centroid of a body with no enclosed region. It is never
// a zero value: a zero volume and "not a solid" are different answers.
var ErrNotSolid = errors.New("decad: body is not a solid")

// ErrDegenerate is returned for an input with no usable geometry: a zero or
// invalid transform, a zero pull direction, a zero wall-thickness tool, a
// draft allowance at or past 90°, or a non-linear edge named as a revolve
// axis. A boolean returns it only for a genuinely malformed operand (a nil,
// self, or extent-less body) and for the retryable coarse-chording
// tessellation refusal — a valid operand whose boolean output cannot be
// chorded finely enough (a finer tolerance may clear it). A valid but
// unclassifiable contact is [BooleanUnsupportedContact], never ErrDegenerate
// (docs/api-design.md §8 / H2).
var ErrDegenerate = errors.New("decad: degenerate input")

// ErrBooleanFailed is returned when a boolean operation cannot produce a
// result body. A public Union, Cut or Intersect reports it wrapped in a
// [BooleanError]: an empty result carries [BooleanEmpty], an internal
// invariant break [BooleanEvaluatorFailure]. errors.Is(err, ErrBooleanFailed)
// still branches on it; errors.As(err, &be) then be.Code is the fine branch.
var ErrBooleanFailed = errors.New("decad: boolean operation failed")

// ErrInvalidProfile is returned when a feature is handed a profile whose Valid
// is false, whose boundary contains a nil entity, or whose exported snapshot
// fields no longer exactly match one current result from Sketch.Profiles. A
// self-intersecting, degenerate, or caller-altered region is never silently
// swept.
var ErrInvalidProfile = errors.New("decad: profile is not a valid region")

// ErrUnitKind is returned for a units.Value whose Kind is not the one the
// parameter takes: an angle where a length is wanted, or a tolerance that is
// not Dimensionless. It is never a coercion — units are never silently
// relabelled.
var ErrUnitKind = errors.New("decad: wrong unit kind")

// ErrNotFinite is returned for a non-finite units.Value magnitude or r3.Vec
// component handed as a parameter, or when Revolve axis validation derives a
// non-finite value from otherwise finite input. units construction admits a
// non-finite value and only its operations reject one, so the call must: an
// infinite tolerance would turn the verification gate off, and a NaN would
// turn it inside out.
var ErrNotFinite = errors.New("decad: non-finite value")

// ErrResourceLimit is returned when recipe decoding crosses a fixed resource
// ceiling before allocating the corresponding typed collection.
var ErrResourceLimit = errors.New("decad: resource limit exceeded")

// ErrUnsupported is returned when the recipe records the intent exactly but
// the current evaluator does not build it. Evaluator staging is explicit
// and rejected at the call — never silently approximated or narrowed — and a
// rejected operation leaves the recipe and the document untouched. See
// docs/evaluator-design.md §2. A public Union, Cut or Intersect wraps a valid
// but unclassifiable contact in a [BooleanError] carrying
// [BooleanUnsupportedContact], but an operand this evaluator cannot tessellate
// at all (a revolve body, or a Faceted operand coarser than the pair tolerance)
// is a capability limit reached before any contact — it passes through as a
// plain ErrUnsupported, not a [BooleanError]. errors.Is(err, ErrUnsupported)
// branches on both.
var ErrUnsupported = errors.New("decad: not supported by the current evaluator")

// ErrInvalidRecipe is returned when stored recipe data violates the recipe
// wire contract. It covers malformed envelopes, unknown or duplicate fields,
// and invalid recipe content. A [RecipeError] carries the failing path and
// preserves this identity for errors.Is.
var ErrInvalidRecipe = errors.New("decad: invalid recipe")

// ErrUnsupportedRecipeVersion is returned when a complete recipe envelope
// names a version this package cannot interpret. The decoder never ignores
// fields from a newer version.
var ErrUnsupportedRecipeVersion = errors.New("decad: unsupported recipe version")

// BooleanErrorCode is the branchable fine reason a public boolean operation
// failed, read from a [BooleanError] with errors.As. It draws the line the
// wrapped sentinel is too coarse to draw: the three codes separate a caller's
// three moves (docs/api-design.md §8 / H2).
type BooleanErrorCode int

const (
	// BooleanEmpty is a NORMAL geometric outcome: the operation encloses no
	// volume — a disjoint Intersect, an all-removing Cut. The model is sound and
	// the operation asked for nothing; change the geometry or drop the call. The
	// [BooleanError] wraps [ErrBooleanFailed].
	BooleanEmpty BooleanErrorCode = iota
	// BooleanUnsupportedContact is a VALID model whose operands meet in a contact
	// this evaluator cannot classify from the tessellated chords: a curved-surface
	// tangency or near-contact, a coplanar face-on-face overlap, a grazing edge,
	// or an isolated-point pinch. The input names a real solid and the refusal is
	// the evaluator's reach, so the [BooleanError] wraps [ErrUnsupported], never
	// [ErrDegenerate]. Choose a construction that does not lean on a tangent
	// contact, or wait for a later evaluator.
	BooleanUnsupportedContact
	// BooleanEvaluatorFailure is an internal invariant break: the stitched
	// boundary did not close, a split line was not found, a chain dangled. It is a
	// bug to file. The [BooleanError] wraps [ErrBooleanFailed].
	BooleanEvaluatorFailure
)

// BooleanError is the typed failure of [Union], [Cut] or [Intersect]
// (docs/api-design.md §8 / H2). It names the operation, the operands as the
// recorded Step would list them (Inputs; [target, tool] for Cut), and a
// branchable Code. It wraps the §12 sentinel errors.Is already branches on, so
// compatibility holds: errors.Is(err, [ErrBooleanFailed]) holds for
// [BooleanEmpty] and [BooleanEvaluatorFailure], and errors.Is(err,
// [ErrUnsupported]) for [BooleanUnsupportedContact]. errors.As(err, &be) then
// be.Code is the fine branch.
//
// A valid but unclassifiable coplanar / tangent / grazing / isolated-point
// contact is [BooleanUnsupportedContact]. [ErrDegenerate] stays reserved for a
// genuinely malformed operand and for the retryable coarse-chording
// tessellation refusal; a whole-operand tessellation-staging [ErrUnsupported]
// (a revolve operand, or a Faceted operand coarser than the pair tolerance)
// passes through plain — none of these three is a BooleanError.
type BooleanError struct {
	Op     OpKind
	Inputs []StepRef
	Code   BooleanErrorCode
	msg    string
	err    error
}

// Error reports the operation and the underlying human-readable reason.
func (e *BooleanError) Error() string {
	return fmt.Sprintf("decad: %s boolean: %s", e.Op, e.msg)
}

// Unwrap returns the wrapped §12 sentinel so errors.Is keeps branching on it.
func (e *BooleanError) Unwrap() error { return e.err }
