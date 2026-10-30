package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

// placedOperandDepartures reads the departures of body's built faces. A
// faceted payload states its own (facetedrecord.PlacedTagDeparture) and an
// operand at the identity placement has none, so both answer nil. Any other
// operand is rebuilt from its record at the identity placement, in a scratch
// document, and each placed tag is charged against the exact image of its
// unplaced twin under the operand's placement (facetedrecord.PlacedBuiltDeparture).
// m is the operand's own mesh, read through tagReach. A rebuild that fails,
// or whose faces do not match the placed body's one for one in surface kind,
// states no departure for any face, so none of them keeps a tag.
func placedOperandDepartures(ctx context.Context, body *topo.Body, m *topo.Mesh) (placedDepartures, error) {
	if body == nil || body.Payload == nil {
		return placedDepartures{}, nil
	}
	if _, faceted := body.Payload.(facetedrecord.Payload); faceted {
		return placedDepartures{}, nil
	}
	composed := body.Payload.Transform()
	if composed == r3.Identity() {
		return placedDepartures{}, nil
	}
	reach, err := tagReach(m)
	if err != nil {
		return placedDepartures{}, err
	}
	faces := topobuild.Faces(body)
	out := placedDepartures{departure: make([]float64, len(faces)), stated: make([]bool, len(faces))}
	env := topo.Env{Doc: &topo.Document{}, Ref: body.Origin.Producer, Alloc: scratchAlloc{}}
	twin, err := body.Payload.Placed(ctx, env, r3.Identity())
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return placedDepartures{}, ctxErr
		}
		return out, nil
	}
	unplaced := topobuild.Faces(twin)
	if len(unplaced) != len(faces) {
		return out, nil
	}
	for i, f := range faces {
		u := unplaced[i]
		if f.Surface == nil || u.Surface == nil || f.Surface.Kind() != u.Surface.Kind() {
			continue
		}
		out.departure[i], out.stated[i] = facetedrecord.PlacedBuiltDeparture(u.Surface, f.Surface, composed,
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
// package: placedOperandDepartures reads its face tags and drops it.
type scratchAlloc struct{}

func (scratchAlloc) NewBody() *topo.Body           { return &topo.Body{} }
func (scratchAlloc) NewLump() *topo.Lump           { return &topo.Lump{} }
func (scratchAlloc) NewFaceShell() *topo.FaceShell { return &topo.FaceShell{} }
func (scratchAlloc) NewFace() *topo.Face           { return &topo.Face{} }
func (scratchAlloc) NewLoop() *topo.Loop           { return &topo.Loop{} }
func (scratchAlloc) NewEdge() *topo.Edge           { return &topo.Edge{} }
func (scratchAlloc) NewVertex() *topo.Vertex       { return &topo.Vertex{} }
