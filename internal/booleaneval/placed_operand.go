package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// placedDepartures is how far the tags of a placed analytic operand's built
// faces lie from their true surfaces (agent-docs/faceted-source-retention-design.md
// §3.5), by the face's position in topobuild.Faces. A nil value charges
// nothing: the operand is faceted or unplaced.
type placedDepartures struct {
	departure []float64
	stated    []bool
}

// operandTwin is the face tags of a placed analytic operand rebuilt at the
// identity placement, by position in topobuild.Faces.
type operandTwin struct {
	// placed is false for an operand charged nothing: no payload, a faceted
	// payload, or the identity placement.
	placed bool
	// tags is nil when the rebuild failed or its faces do not pair one for
	// one with the placed body's.
	tags []surfacegeom.Surface
}

// operandTwins caches each operand's twin across the attempts of one
// Evaluate, so the shifted retry does not rebuild it again.
type operandTwins struct {
	done [2]bool
	twin [2]operandTwin
}

// of returns operand k's twin, rebuilding body at most once.
func (c *operandTwins) of(ctx context.Context, k int, body *topo.Body) (operandTwin, error) {
	if c.done[k] {
		return c.twin[k], nil
	}
	twin, err := unplacedTwin(ctx, body)
	if err != nil {
		return operandTwin{}, err
	}
	c.done[k], c.twin[k] = true, twin
	return twin, nil
}

// unplacedTwin rebuilds body from its record at the identity placement, in a
// scratch document. A faceted payload states its own departure
// (facetedrecord.PlacedTagDeparture) and an operand at the identity placement
// has none, so neither is rebuilt. A rebuild that fails, or whose faces do not
// match the placed body's in number, leaves no tags.
func unplacedTwin(ctx context.Context, body *topo.Body) (operandTwin, error) {
	if body == nil || body.Payload == nil {
		return operandTwin{}, nil
	}
	if _, faceted := body.Payload.(facetedrecord.Payload); faceted {
		return operandTwin{}, nil
	}
	if body.Payload.Transform() == r3.Identity() {
		return operandTwin{}, nil
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: body.Origin.Producer, Alloc: scratchAlloc{}}
	twin, err := body.Payload.Placed(ctx, env, r3.Identity())
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return operandTwin{}, ctxErr
		}
		return operandTwin{placed: true}, nil
	}
	faces := topobuild.Faces(twin)
	if len(faces) != len(topobuild.Faces(body)) {
		return operandTwin{placed: true}, nil
	}
	tags := make([]surfacegeom.Surface, len(faces))
	for i, f := range faces {
		tags[i] = f.Surface
	}
	return operandTwin{placed: true, tags: tags}, nil
}

// placedOperandDepartures charges each built face of body against the exact
// image of its twin's tag under the operand's placement
// (facetedrecord.PlacedBuiltDeparture). m is the operand's own mesh, read
// through tagReach. A twin with no tags states no departure for any face, so
// none of them keeps a tag.
func placedOperandDepartures(body *topo.Body, twin operandTwin, m *topo.Mesh) (placedDepartures, error) {
	if !twin.placed {
		return placedDepartures{}, nil
	}
	reach, err := tagReach(m)
	if err != nil {
		return placedDepartures{}, err
	}
	faces := topobuild.Faces(body)
	out := placedDepartures{departure: make([]float64, len(faces)), stated: make([]bool, len(faces))}
	if twin.tags == nil {
		return out, nil
	}
	composed := body.Payload.Transform()
	for i, f := range faces {
		u := twin.tags[i]
		if f.Surface == nil || u == nil || f.Surface.Kind() != u.Kind() {
			continue
		}
		out.departure[i], out.stated[i] = facetedrecord.PlacedBuiltDeparture(u, f.Surface, composed,
			m.Vertices, reach)
	}
	return out, nil
}

// charge returns g with the departure of the built face at position i added to
// its normal bound. A group with no tag, a retained or denoted face (each
// states its own departure), and an unplaced operand keep g. A face whose
// departure is not stated keeps its provenance and no tag.
func (d placedDepartures) charge(g facetedrecord.Group, f *topo.Face, i int) facetedrecord.Group {
	if d.departure == nil || g.Surface == nil || f.RetainedTag || f.Denoted != nil {
		return g
	}
	if i >= len(d.stated) || !d.stated[i] {
		return facetedrecord.Group{Origins: g.Origins, Upstream: g.Upstream, Source: g.Source}
	}
	g.NormalBound = min(2, proofbound.AbsSumUpper(g.NormalBound, d.departure[i]))
	return g
}

// scratchAlloc allocates the bare nodes of a rebuild that never leaves this
// package: unplacedTwin reads its face tags and drops it.
type scratchAlloc struct{}

func (scratchAlloc) NewBody() *topo.Body           { return &topo.Body{} }
func (scratchAlloc) NewLump() *topo.Lump           { return &topo.Lump{} }
func (scratchAlloc) NewFaceShell() *topo.FaceShell { return &topo.FaceShell{} }
func (scratchAlloc) NewFace() *topo.Face           { return &topo.Face{} }
func (scratchAlloc) NewLoop() *topo.Loop           { return &topo.Loop{} }
func (scratchAlloc) NewEdge() *topo.Edge           { return &topo.Edge{} }
func (scratchAlloc) NewVertex() *topo.Vertex       { return &topo.Vertex{} }
