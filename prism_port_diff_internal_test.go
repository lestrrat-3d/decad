package decad

// TestPrismPortMatchesRootBuilder is H3.0b's stage-A differential check: it
// rebuilds bodies the root builder (evalPrismContext) made through the public
// API with the ported builder (internal/prismrecord.Build) on bare nodes, and
// compares every measurement, face, edge and vertex field bit for bit. Stage B
// deletes the root builder and this test with it.

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func portDiffHoledPlate(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	body, err := Extrude(t.Context(), doc, s, prof, Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

// portFaceData is a face node with its links and identity tokens cleared:
// the fields both builders compute from the record alone.
func portFaceData(n topo.Face) topo.Face {
	n.Body, n.Loops, n.Owner = nil, nil, nil
	return n
}

// portEdgeData is an edge node with its links and identity tokens cleared.
func portEdgeData(n topo.Edge) topo.Edge {
	n.Start, n.End, n.Faces, n.Owner = nil, nil, nil, nil
	n.Level, n.Denot = denotation.LevelToken{}, denotation.CurveToken{}
	return n
}

// portTokens checks that the port minted its level and curve identities in
// the root builder's order: each pair is zero together, carries the same
// plane or motion, and its two ids differ by one fixed offset per kind (the
// two documents' counters started apart).
type portTokens struct {
	levelSet, curveSet bool
	levelOff           denotation.LevelID
	curveOff           int
}

func (p *portTokens) level(t *testing.T, want, got denotation.LevelToken) {
	t.Helper()
	require.Equal(t, want.ID == 0, got.ID == 0)
	require.Equal(t, want.Origin, got.Origin)
	require.Equal(t, want.Normal, got.Normal)
	if want.ID == 0 {
		return
	}
	if !p.levelSet {
		p.levelSet, p.levelOff = true, want.ID-got.ID
		return
	}
	require.Equal(t, p.levelOff, want.ID-got.ID, `level ids in build order`)
}

func (p *portTokens) curve(t *testing.T, want, got denotation.CurveToken) {
	t.Helper()
	require.Equal(t, want.ID == 0, got.ID == 0)
	require.Equal(t, want.Xform, got.Xform)
	if want.ID == 0 {
		return
	}
	if !p.curveSet {
		p.curveSet, p.curveOff = true, int(want.ID-got.ID)
		return
	}
	require.Equal(t, p.curveOff, int(want.ID-got.ID), `curve ids in build order`)
}

func (p *portTokens) vertex(t *testing.T, want, got *topo.Vertex) {
	t.Helper()
	require.Equal(t, want.Position, got.Position)
	require.Equal(t, want.Bound, got.Bound)
	p.level(t, want.Level, got.Level)
	p.curve(t, want.Denot, got.Denot)
}

func TestPrismPortMatchesRootBuilder(t *testing.T) {
	t.Parallel()
	doc := New()
	axis, ok := r3.NewVec(1, 1, 0).Normalize()
	require.True(t, ok)
	rot, err := r3.RotationAround(r3.NewVec(1, 2, 3), axis, units.Radians(0.7))
	require.NoError(t, err)
	box := internalBoxBody(t, doc, 0, 0, 20, 10, 5)
	turned, err := Transform(t.Context(), internalBoxBody(t, doc, 1, 1, 7, 9, 3), rot)
	require.NoError(t, err)
	filleted, err := Fillet(t.Context(), internalBoxBody(t, doc, -6, -6, 6, 6, 4), Edges(ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(2))
	require.NoError(t, err)
	bodies := map[string]*Body{
		"box":      box,
		"circle":   internalCircleBody(t, New(), 3, 4, 2, Distance{D: units.Millimeters(7), Dir: Along}),
		"halfDisc": internalHalfDiscPrism(t, New()),
		"sheet":    internalHoledSheetBody(t, New()),
		"holed":    portDiffHoledPlate(t, New()),
		"turned":   turned,
		"filleted": filleted,
	}
	for name, b := range bodies {
		t.Run(name, func(t *testing.T) {
			pp, ok := b.n.Payload.(prismPayload)
			require.True(t, ok, `%s is a prism`, name)
			ndoc := &topo.Document{}
			env := topo.Env{Doc: ndoc, Ref: b.n.Origin.Producer, Alloc: topotest.Alloc{}}
			rec := pp.Payload
			rec.Walks = nil
			nb, err := prismrecord.Build(t.Context(), env, rec, freeform.NewFreeformWork())
			require.NoError(t, err)

			require.Same(t, ndoc, nb.Doc)
			require.Equal(t, b.n.Volume, nb.Volume)
			require.Equal(t, b.n.Area, nb.Area)
			require.Equal(t, b.n.Centroid, nb.Centroid)
			require.Equal(t, b.n.Bounds, nb.Bounds)
			require.Equal(t, b.n.Origin, nb.Origin)
			require.Equal(t, b.n.Kind, nb.Kind)
			require.Equal(t, b.n.Solid, nb.Solid)
			got, ok := nb.Payload.(prismrecord.Payload)
			require.True(t, ok, `the port stores its record`)
			require.Equal(t, pp.Xform, got.Xform)
			require.Equal(t, pp.Profile, got.Profile)
			require.NotNil(t, got.Walks, `the port publishes its walk resolution`)
			require.Len(t, nb.Lumps, len(b.n.Lumps))
			for i, l := range b.n.Lumps {
				require.Len(t, nb.Lumps[i].Shells, len(l.Shells))
				for j, s := range l.Shells {
					require.Equal(t, s.Open, nb.Lumps[i].Shells[j].Open)
					require.Equal(t, s.Void, nb.Lumps[i].Shells[j].Void)
					require.Len(t, nb.Lumps[i].Shells[j].Faces, len(s.Faces))
				}
			}

			tokens := &portTokens{}
			faces, nfaces := b.Faces(), topobuild.Faces(nb)
			require.Len(t, nfaces, len(faces))
			for i, f := range faces {
				require.Equal(t, portFaceData(f.n), portFaceData(*nfaces[i]), `face %d`, i)
				require.Same(t, nb, nfaces[i].Body, `face %d body`, i)
				require.Len(t, nfaces[i].Loops, len(f.n.Loops), `face %d loops`, i)
				for k, l := range f.n.Loops {
					nl := nfaces[i].Loops[k]
					require.Equal(t, l.Outer, nl.Outer)
					require.Len(t, nl.CoEdges, len(l.CoEdges))
					for c, ce := range l.CoEdges {
						require.Equal(t, ce.Forward, nl.CoEdges[c].Forward)
						require.Equal(t, portEdgeData(*ce.Edge), portEdgeData(*nl.CoEdges[c].Edge))
					}
				}
			}
			edges, nedges := b.Edges(), topobuild.Edges(nb)
			require.Len(t, nedges, len(edges))
			for i, e := range edges {
				require.Equal(t, portEdgeData(e.n), portEdgeData(*nedges[i]), `edge %d`, i)
				require.Len(t, nedges[i].Faces, len(e.n.Faces), `edge %d faces`, i)
				tokens.level(t, e.n.Level, nedges[i].Level)
				tokens.curve(t, e.n.Denot, nedges[i].Denot)
				tokens.vertex(t, e.n.Start, nedges[i].Start)
				tokens.vertex(t, e.n.End, nedges[i].End)
			}
		})
	}
}
