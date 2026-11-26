package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// FilletChord rounds the edges sel selects with an arc whose chord, the
// straight line between its two tangent edges, is chord long, returning the
// new body and retiring b (agent-docs/modify-design.md Part A).
//
// Where the faces meet at the angle φ across the corner (the interior angle of a
// convex edge, the open wedge of a concave one) the arc turns π − φ, so its
// radius is chord/(2·cos(φ/2)), which is chord/√2 on a right angle.
// FilletChord states that radius and builds the result as [Fillet] does, so the
// receivers, routes and refusals of Fillet apply.
//
// Every selected edge MUST lie between two planar faces, or be circular between a plane and a cylinder, cone or sphere, and may be convex or
// concave: a concave edge's fillet adds the round's material to the corner. Any
// other edge is ErrUnsupported. All the selected edges MUST give the same radius,
// within a relative 1e-12, which equal dihedrals do; edges at different angles
// take separate calls, and a selection that mixes them is ErrUnsupported. chord
// MUST be a positive length; a zero or negative one is ErrDegenerate or
// ErrNegativeMagnitude, and a wrong unit kind is ErrUnitKind. A nil context,
// body or selector is ErrDegenerate. Every failure leaves b live and the
// document unchanged.
func FilletChord(b *Body, sel EdgeSelector, chord units.Value) *FilletChordCommand {
	c := &FilletChordCommand{b: b, sel: copyEdgeSelector(sel), chord: chord}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// FilletChordCommand rounds edges of a body by chord length. Build it with
// [FilletChord]. It has no method.
type FilletChordCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	sel   EdgeSelector    // copied at FilletChord
	chord units.Value     // the chord length, gated at Do
}

// Do rounds the edges and retires the body, or returns the error of the first
// gate that refuses, leaving the body live. A nil ctx is ErrDegenerate, and a
// second call returns ErrConsumed. Its errors carry [OperationFillet].
func (c *FilletChordCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationFillet, c.run)
}

func (c *FilletChordCommand) run(ctx context.Context) (*Body, error) {
	return filletChord(ctx, c.b, c.sel, c.chord)
}

// filletChord is FilletChord's implementation. ctx is not nil, and the errors
// it returns carry no operation tag (agent-docs/command-api-design.md §11).
func filletChord(ctx context.Context, b *Body, sel EdgeSelector, chord units.Value) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	cmm, err := sectionrecord.MagnitudeIn(chord, units.Length, units.Millimeter, "the fillet chord")
	if err != nil {
		return nil, err
	}
	if cmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-chord fillet is the body the caller already holds`, ErrDegenerate)
	}
	// A selector Fillet would refuse is left to Fillet, so the error is its own.
	q, ok := sel.(*EdgeQuery)
	if !ok || q == nil {
		return fillet(ctx, b, sel, chord, modifyoption.FilletConfig{})
	}
	edges, err := q.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	var radius float64
	for i, e := range edges {
		phi, err := e.planarDihedral("a chord-length fillet")
		if err != nil {
			return nil, err
		}
		r := cmm / (2 * math.Cos(phi/2))
		if i == 0 {
			radius = r
		} else if !sameWithinDihedralTolerance(radius, r) {
			return nil, fmt.Errorf(`%w: the selected edges meet at different angles, so they need different radii`, ErrUnsupported)
		}
	}
	return fillet(ctx, b, sel, units.Millimeters(radius), modifyoption.FilletConfig{})
}
