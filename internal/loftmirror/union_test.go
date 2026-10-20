package loftmirror_test

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmirror"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// meshes returns, for each body it is asked about, a mesh that restates the
// body's loft record with the given proof values, and cancels the context
// once both operands are meshed so a run that clears every gate stops in the
// facet loop instead of building a body.
type meshes struct {
	cancel                context.CancelFunc
	calls                 int
	boundary, volume      bool
	bound, volume2, area2 float64
	edit                  func(call int, m *topo.Mesh)
}

func (m *meshes) VerifyAll(_ context.Context, b *topo.Body, _ float64) (*topo.Mesh, error) {
	m.calls++
	lp := b.Payload.(loftrecord.Payload)
	mesh := &topo.Mesh{
		Vertices: slices.Clone(lp.Verts), Triangles: slices.Clone(lp.Tris),
		Source:     make([]*topo.Face, len(lp.Tris)),
		BoundaryOK: m.boundary, SymDiffOK: m.volume,
		Bound: m.bound, VolSymDiff: m.volume2, AreaSlack: m.area2,
	}
	for i := range mesh.Source {
		mesh.Source[i] = &topo.Face{Origins: []provenance.Ref{{Producer: 1, Role: "wall"}}}
	}
	if m.edit != nil {
		m.edit(m.calls, mesh)
	}
	if m.calls == 2 {
		m.cancel()
	}
	return mesh, nil
}

func square() momentinput.Profile {
	corners := []sectionrecord.Point2{{}, {U: 4}, {U: 4, V: 4}, {V: 4}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

// pair is two lofts that clear every gate of the mirror Union: one source,
// XY-aligned frames on the Z axis, an upper loft placed by the reflection in
// the end plane, equal sections, three stations, and cap triangles that
// oppose.
func pair(t *testing.T) (lower, upper loftrecord.Payload) {
	t.Helper()
	frame := func(z float64) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{Z: z}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		return f
	}
	lower = loftrecord.Payload{
		SourceIdentity: new(loftrecord.SourceIdentity), AuthenticatedReconstruction: true,
		Profile0: square(), Profile1: square(), Frame0: frame(0), Frame1: frame(5), Alignment: []int{0, 0},
		Xform: r3.Identity(), Walls: 6, CapStartCount: 1,
		Verts: []r3.Vec{{}, {X: 4}, {X: 4, Y: 4}, {Z: 5}, {X: 4, Z: 5}, {X: 4, Y: 4, Z: 5}},
		Tris:  [][3]int{{0, 1, 4}, {0, 4, 3}, {1, 2, 5}, {1, 5, 4}, {2, 0, 3}, {2, 3, 5}, {0, 2, 1}, {3, 4, 5}},
	}
	reflection, err := r3.Reflection(lower.Frame1)
	require.NoError(t, err)
	upper = lower
	upper.Xform = reflection
	upper.Alignment = slices.Clone(lower.Alignment)
	upper.Verts = slices.Clone(lower.Verts)
	upper.Tris = slices.Clone(lower.Tris)
	upper.Tris[7] = [3]int{3, 5, 4}
	return lower, upper
}

func solid(p loftrecord.Payload) *topo.Body {
	return &topo.Body{Solid: true, Payload: p,
		Bounds: measurement.Box{Min: r3.Vec{}, Max: r3.Vec{X: 4, Y: 4, Z: 5}}}
}

func TestUnionDeclinesWhatIsNotAMirroredLoftPair(t *testing.T) {
	t.Parallel()

	lowerFrame := func(z float64) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{Z: z}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		return f
	}
	skewed, err := r3.NewFrame(r3.Vec{X: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	tests := map[string]func(lower, upper *loftrecord.Payload){
		"lower is moved":             func(l, _ *loftrecord.Payload) { l.Xform = upperXform(t) },
		"lower is not reconstructed": func(l, _ *loftrecord.Payload) { l.AuthenticatedReconstruction = false },
		"upper is not reconstructed": func(_, u *loftrecord.Payload) { u.AuthenticatedReconstruction = false },
		"lower is a sheet":           func(l, _ *loftrecord.Payload) { l.SurfaceResult = true },
		"upper is a sheet":           func(_, u *loftrecord.Payload) { u.SurfaceResult = true },
		"lower has no source":        func(l, _ *loftrecord.Payload) { l.SourceIdentity = nil },
		"neither has a source": func(l, u *loftrecord.Payload) {
			l.SourceIdentity, u.SourceIdentity = nil, nil
		},
		"both frames off the axis": func(l, u *loftrecord.Payload) {
			l.Frame0, u.Frame0 = skewed, skewed
		},
		"both frames turned": func(l, u *loftrecord.Payload) {
			l.Frame1, u.Frame1 = turned, turned
		},
		"other source":              func(_, u *loftrecord.Payload) { u.SourceIdentity = new(loftrecord.SourceIdentity) },
		"start frame off the axis":  func(l, _ *loftrecord.Payload) { l.Frame0 = skewed },
		"end frame off the axis":    func(l, _ *loftrecord.Payload) { l.Frame1 = skewed },
		"start frame turned":        func(l, _ *loftrecord.Payload) { l.Frame0 = turned },
		"end frame not above start": func(l, _ *loftrecord.Payload) { l.Frame1 = lowerFrame(-5) },
		"start frames differ":       func(_, u *loftrecord.Payload) { u.Frame0 = lowerFrame(1) },
		"end frames differ":         func(_, u *loftrecord.Payload) { u.Frame1 = lowerFrame(6) },
		"start planes differ":       func(_, u *loftrecord.Payload) { u.Plane0.Origin.X = 1 },
		"end planes differ":         func(_, u *loftrecord.Payload) { u.Plane1.Origin.X = 1 },
		"alignments differ":         func(_, u *loftrecord.Payload) { u.Alignment = []int{0, 1} },
		"upper is not the reflection": func(_, u *loftrecord.Payload) {
			u.Xform = r3.Identity()
		},
		"start sections differ": func(_, u *loftrecord.Payload) {
			u.Profile0.Outer.Segments = u.Profile0.Outer.Segments[:3]
		},
		"end sections differ": func(_, u *loftrecord.Payload) {
			u.Profile1.Outer.Segments = u.Profile1.Outer.Segments[:3]
		},
	}
	for name, edit := range tests {
		lower, upper := pair(t)
		edit(&lower, &upper)
		cancelled, cancel := context.WithCancel(t.Context())
		m := &meshes{cancel: cancel, boundary: true, volume: true}
		got, ok, err := loftmirror.Union(cancelled, meshbool.OpUnion, topo.Env{}, m, solid(lower), solid(upper))
		cancel()
		require.NoError(t, err, name)
		require.False(t, ok, name)
		require.Nil(t, got, name)
		require.Zero(t, m.calls, "%s: a pair that is not the class is never meshed", name)
	}

	lower, upper := pair(t)
	for name, tc := range map[string]struct {
		op   meshbool.OperationKind
		a, b *topo.Body
	}{
		"a cut":             {meshbool.OpCut, solid(lower), solid(upper)},
		"an intersection":   {meshbool.OpIntersect, solid(lower), solid(upper)},
		"first not solid":   {meshbool.OpUnion, &topo.Body{Payload: lower}, solid(upper)},
		"second not solid":  {meshbool.OpUnion, solid(lower), &topo.Body{Payload: upper}},
		"first not a loft":  {meshbool.OpUnion, &topo.Body{Solid: true}, solid(upper)},
		"second not a loft": {meshbool.OpUnion, solid(lower), &topo.Body{Solid: true}},
	} {
		_, ok, err := loftmirror.Union(t.Context(), tc.op, topo.Env{}, &meshes{cancel: func() {}}, tc.a, tc.b)
		require.NoError(t, err, name)
		require.False(t, ok, name)
	}
}

func upperXform(t *testing.T) r3.Transform {
	t.Helper()
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	return move
}

func TestUnionAcceptsEitherOrder(t *testing.T) {
	t.Parallel()

	// With the moved loft first, the pair is swapped and still clears every gate.
	lower, upper := pair(t)
	cancelled, cancel := context.WithCancel(t.Context())
	m := &meshes{cancel: cancel, boundary: true, volume: true, bound: 0.001, volume2: 0.002, area2: 0.003}
	_, ok, err := loftmirror.Union(cancelled, meshbool.OpUnion, topo.Env{}, m, solid(upper), solid(lower))
	cancel()
	require.True(t, ok)
	require.ErrorIs(t, err, context.Canceled, "the run reached the facet loop")
	require.Equal(t, 2, m.calls)
}

const (
	stationsMsg = "lack matching cap and wall stations"
	proofMsg    = "no matching boundary and volume proof"
	finiteMsg   = "proof is not finite"
)

func TestUnionRefusesAMalformedMirrorPair(t *testing.T) {
	t.Parallel()

	tests := map[string]struct {
		edit func(lower, upper *loftrecord.Payload)
		msg  string
	}{
		"odd vertex count": {func(l, u *loftrecord.Payload) {
			l.Verts = append(l.Verts, r3.Vec{})
			u.Verts = append(u.Verts, r3.Vec{})
		}, stationsMsg},
		"two stations": {func(l, u *loftrecord.Payload) {
			l.Verts, u.Verts = l.Verts[:4], u.Verts[:4]
			l.Walls, u.Walls = 4, 4
		}, stationsMsg},
		"upper vertex count": {func(_, u *loftrecord.Payload) { u.Verts = u.Verts[:5] }, stationsMsg},
		"wall count":         {func(l, u *loftrecord.Payload) { l.Walls, u.Walls = 5, 5 }, stationsMsg},
		"upper wall count":   {func(_, u *loftrecord.Payload) { u.Walls = 5 }, stationsMsg},
		"no start cap":       {func(l, u *loftrecord.Payload) { l.CapStartCount, u.CapStartCount = 0, 0 }, stationsMsg},
		"upper start cap":    {func(_, u *loftrecord.Payload) { u.CapStartCount = 2 }, stationsMsg},
		"triangle counts":    {func(_, u *loftrecord.Payload) { u.Tris = u.Tris[:7] }, stationsMsg},
		"no end cap": {func(l, u *loftrecord.Payload) {
			l.Tris, u.Tris = l.Tris[:7], u.Tris[:7]
		}, stationsMsg},
		"cap stations differ": {func(_, u *loftrecord.Payload) { u.Verts[4].Z = 5.5 }, "cap stations differ"},
		"caps do not oppose":  {func(_, u *loftrecord.Payload) { u.Tris[7] = [3]int{3, 4, 5} }, "cap triangles do not oppose"},
	}
	for name, tc := range tests {
		lower, upper := pair(t)
		tc.edit(&lower, &upper)
		cancelled, cancel := context.WithCancel(t.Context())
		m := &meshes{cancel: cancel, boundary: true, volume: true}
		_, ok, err := loftmirror.Union(cancelled, meshbool.OpUnion, topo.Env{}, m, solid(lower), solid(upper))
		cancel()
		require.True(t, ok, name)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
		require.Zero(t, m.calls, "%s: the record gates run before any mesh", name)
	}
}

// The volume and area terms are absolute sums, so a negative input cannot make
// either one negative; only the bound, a plain maximum, can be.
func TestUnionNeedsMatchingProvenMeshes(t *testing.T) {
	t.Parallel()

	tests := map[string]struct {
		mesh func(m *meshes)
		msg  string
	}{
		"lower boundary unproven": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) { mesh.BoundaryOK = mesh.BoundaryOK && call != 1 }
		}, proofMsg},
		"upper boundary unproven": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) { mesh.BoundaryOK = mesh.BoundaryOK && call != 2 }
		}, proofMsg},
		"lower volume unproven": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) { mesh.SymDiffOK = mesh.SymDiffOK && call != 1 }
		}, proofMsg},
		"upper volume unproven": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) { mesh.SymDiffOK = mesh.SymDiffOK && call != 2 }
		}, proofMsg},
		"lower vertices differ": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) {
				if call == 1 {
					mesh.Vertices[0].X = 9
				}
			}
		}, proofMsg},
		"upper triangles differ": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) {
				if call == 2 {
					mesh.Triangles[0] = [3]int{0, 0, 0}
				}
			}
		}, proofMsg},
		"lower sources short": {func(m *meshes) {
			m.edit = func(call int, mesh *topo.Mesh) {
				if call == 1 {
					mesh.Source = mesh.Source[:1]
				}
			}
		}, proofMsg},
		"bound not finite":  {func(m *meshes) { m.bound = math.NaN() }, finiteMsg},
		"bound infinite":    {func(m *meshes) { m.bound = math.Inf(1) }, finiteMsg},
		"bound negative":    {func(m *meshes) { m.bound = -1 }, finiteMsg},
		"volume not finite": {func(m *meshes) { m.volume2 = math.Inf(1) }, finiteMsg},
		"area not finite":   {func(m *meshes) { m.area2 = math.Inf(1) }, finiteMsg},
	}
	for name, tc := range tests {
		lower, upper := pair(t)
		cancelled, cancel := context.WithCancel(t.Context())
		m := &meshes{cancel: cancel, boundary: true, volume: true, bound: 0.001, volume2: 0.002, area2: 0.003}
		tc.mesh(m)
		_, ok, err := loftmirror.Union(cancelled, meshbool.OpUnion, topo.Env{}, m, solid(lower), solid(upper))
		cancel()
		require.True(t, ok, name)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}
}
