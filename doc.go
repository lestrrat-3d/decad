// Package decad is a headless CAD engine for Go: the 3D modeling layer above
// the [sketch] 2D constraint engine and the [r3] coordinate-math layer, with
// every scalar quantity a typed [units] value.
//
// # Why this exists
//
// decad is built to be driven by a coding agent as a verification step BEFORE
// it commits to writing real CAD software code (an Autodesk Fusion add-in, say).
// 3D modeling is easy to get subtly wrong — a profile that does not sweep the
// way you expect, a feature that leaves a self-intersecting or non-watertight
// body, a wall thinner than the tool that has to cut it, two components that
// interfere. Discovering that inside Fusion, after the script has run, is
// expensive.
//
// Instead, an agent reproduces the part here first and interrogates it
// programmatically — is the body watertight, what is its volume and bounding
// box, does it self-intersect, do these two bodies collide — and only carries
// the plan into the CAD package once the geometry is proven sound.
//
// # Quickstart
//
// The canonical loop is sketch → model → verify → gate. Build and solve a 2D
// profile in [sketch], turn it into a body with a feature verb, then gate the
// live document on [Report.Trustworthy]:
//
//	w := sketch.NewWorld()
//	s, err := w.CreateSketch(w.XY())
//	if err != nil {
//		return err
//	}
//	rect := s.CreateRectangle(0, 0, 100, 60)
//	s.Fix(rect.A)
//	if _, err := s.Solve(context.Background()); err != nil {
//		return err
//	}
//
//	doc := decad.New()
//	body, err := doc.Extrude(s, s.Profiles()[0],
//		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
//	if err != nil {
//		// Every failure wraps a sentinel from errors.go — branch with
//		// errors.Is(err, decad.ErrUnsupported), decad.ErrCardinality, etc.
//		return err
//	}
//
//	// Verify can fail (a cancelled context) and return a NIL report, so gate
//	// only after checking err — report.Trustworthy() would panic otherwise.
//	report, err := doc.Verify(context.Background())
//	if err != nil {
//		return err
//	}
//	if !report.Trustworthy() {
//		// Something is not proven right: report.Status says how severe,
//		// and each report.Bodies entry carries that body's verdict.
//	}
//	_ = body
//
// A runnable version of this loop is Example_decad_quickstart in this package's
// test files. The examples directory holds fuller, feature-specific cases —
// selectors, revolve, the modify ops, booleans, verification, recipe
// serialization and error recovery — each an executable test with a verified
// output block.
//
// # Units and coordinates
//
// Every SCALAR quantity is a typed [units.Value]: distances, angles, radii,
// thicknesses, tolerances, and every measurement and its error bound. A value
// carries its Kind, so a length handed where an angle is wanted is a loud
// [ErrUnitKind], never a silent coercion.
//
// Coordinates are the deliberate exception (core §5.2). A position is a bare
// [r3.Vec] (or a plane-local Point2) in millimetres by convention — not a
// units.Value — and a direction vector is dimensionless. So scalar inputs read
// units.Millimeters(10), while a translation or a selector direction reads
// r3.NewVec(200, 0, 25) with no unit marker.
//
// # Evaluator support
//
// The recipe records intent under a stable design; this evaluator builds a
// subset of it and refuses the rest explicitly (never a wrong-but-confident
// result). The current map, and the sentinel a refused combination returns:
//
//	Extrude       line/circle/arc profile segments            builds
//	  free-form profile segment (spline, ellipse)             ErrUnsupported
//	  WithTaper   nonzero taper angle                         ErrUnsupported
//	Revolve       cylinder / cone / sphere / torus / annulus  builds
//	Union/Cut/Intersect  prism/faceted operands, crossings    builds
//	  faceted operand coarser than the pair tolerance         ErrUnsupported
//	  revolve operand (no tessellator; booleans mesh)         ErrUnsupported
//	  curved-surface tangent, facets never meet               ErrUnsupported
//	  exact coplanar / face-on-face / point contact outside
//	    admitted analytic prism Union                         ErrUnsupported
//	  empty result (disjoint intersect, emptied cut)          ErrBooleanFailed
//	Fillet/Chamfer  straight prism, lateral edges             builds
//	Chamfer       complete prism cap loop(s)                  builds
//	  Fillet of a cap edge (the vertex blend)                 ErrUnsupported
//	  partial or lateral-mixed cap-loop selection             ErrUnsupported
//	  cap-loop setback the radius or sweep cannot name        ErrUnsupported
//	  cap-loop corner whose offset cannot be enclosed         ErrUnsupported
//	  non-prism receiver, or a cap-loop chamfer result        ErrUnsupported
//	Shell         straight prism (tube or cup)                builds
//	  both caps removed from a holed section                  ErrUnsupported
//	Placed        any body this evaluator built               builds
//	Verify        every body; surveys read prisms/revolves/cups/cap blends
//	  a question the evaluator cannot decide                  Status Suspect
//	Tessellate / STL / OBJ  prism, cup, boolean body          builds
//	  revolve payload                                         ErrUnsupported
//	  cap-loop chamfer result                                 ErrUnsupported
//	  boolean body at a tolerance finer than its bound        ErrUnsupported
//
// Options: among the MODEL-CONSTRUCTION verbs, New, Revolve, Fillet and
// Chamfer expose option groups that carry nothing today (they exist so options
// can be added without a signature change); WithShellSense picks a shell's wall
// sense, and WithTaper names an extrude taper — but a nonzero taper is
// [ErrUnsupported], returned before any step is recorded, so the recipe is left
// unchanged. Separately, Verify's options (WithTolerance, WithMinWallThickness,
// WithPullDirection, WithMinRadius, WithClearances) and the STL/OBJ
// WithChordTolerance also take effect.
//
// # Layering
//
//	decad    3D bodies, features, verification   (this module)
//	  |
//	sketch   parametric 2D constraint solving    github.com/lestrrat-3d/sketch
//	  |
//	r3       vectors, frames, rigid transforms   github.com/lestrrat-3d/r3
//	  |
//	units    typed quantities (Value, Kind)      github.com/lestrrat-3d/units
//
// The arrows point down and NEVER back up: decad imports sketch, r3 and units;
// none of them knows decad exists. A 2D question — does this profile close, is this
// sketch fully constrained — is answered by sketch and consumed here as a given.
// decad does not re-derive it.
//
// r3 deliberately excludes shapes ("if it lives in ℝ³ it belongs there; if it
// IS a shape it does not"), and sketch deliberately excludes anything that must
// be computed FROM a solid. decad is the layer both of them were leaving room
// for.
//
// # Design
//
// The public API is landing incrementally against an approved design: what
// this package exports today is the leading edge of that surface, and
// whatever it does not yet export — ultimately the document, features,
// evaluator and verification surface — remains design-only. The
// contract for the whole surface is docs/api-design.md — the core design.
// Companion designs carry its deep ends; CLAUDE.md's Layout table lists every
// design document. Every
// capability it consumes from sketch, r3 and units exists today — no open
// dependency gaps — and nothing that contradicts it may be added to this
// package.
//
// [sketch]: https://github.com/lestrrat-3d/sketch
// [r3]: https://github.com/lestrrat-3d/r3
// [units]: https://github.com/lestrrat-3d/units
package decad
