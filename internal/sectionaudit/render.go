package sectionaudit

// The §5 audit of a fillet's rewritten section (agent-docs/modify-design.md §5)
// runs before any face is built, in the order §4 fixes: S8 (orientation,
// asked first), S6 (no walk
// consumed by its own corners), S7 (no crossing or boundary contact), then S9
// (nesting). Every test is a closed-form fact of decad's own line and arc
// segments, so its verdict is the same under every evaluator — never a residual.
//
// S7 rejects boundary CONTACT as well as crossing, so that the loops the
// rewrite hands S9 are strictly DISJOINT. Two Jordan loops stop being disjoint
// in exactly two ways: their boundaries CROSS, or they merely TOUCH (a
// tangency, or a shared boundary point with no interior crossing). A large
// fillet can pinch the rewritten loops into contact without crossing, so S7
// tests both (segCross for a transversal crossing, segMinDist for a tangency or
// shared point).
//
// Disjoint is not the same as nested: two disjoint Jordan loops are either
// nested OR mutually exterior, and S8 (each loop's own signed area) reads no
// relative position, so it cannot tell the two apart. A large fillet can shrink
// the outer loop past a near-corner hole, leaving the hole in the removed corner
// region — disjoint from every outer segment, yet OUTSIDE the rounded material.
// So S9 (sectionaudit.Nesting) is COMPUTED, not discharged by construction: it
// classifies one point of each hole against the outer loop and each other hole,
// using the same ray-parity walk with direction retries that internal/survey2d/wall_kernel.go runs
// (loopContains). An undecidable containment is S9 ErrUnsupported — the
// evaluator declines rather than guess; a hole PROVEN outside the outer loop, or
// nested inside another hole, is nesting decidably broken — the fillet consumed
// the region the caller's section lived in, so no such body exists and it is an
// S8-family ErrDegenerate (§1 existence test).

type renderedDiagnosticError struct {
	cause   error
	message string
}

func (e *renderedDiagnosticError) Error() string { return e.message }

func (e *renderedDiagnosticError) Unwrap() error { return e.cause }

// RenderCoordinates opts a Fillet or Chamfer failure into the detailed
// diagnostic without changing the shared audit error seen by Shell.
func RenderCoordinates(err error) error {
	diagnostic, ok := err.(*DiagnosticError)
	if !ok {
		return err
	}
	return &renderedDiagnosticError{cause: err, message: diagnostic.Detailed()}
}
