package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// unionBossShell is a rotated boss unioned onto a plate, the receiver
// prismshell.PrismUnionBossShell accepts, with the boss cap it removes.
type unionBossShell struct {
	result      *topo.Body
	fp          facetedPayload
	removed     []*topo.Face
	plate, boss *topo.Body
	t           float64
	inward      bool
	tDelta      float64
}

func unionBossShellFixture(t *testing.T, plateZ0, plateHeight, bossZ0, bossHeight float64) *unionBossShell {
	t.Helper()
	doc := New()
	plate := internalBoxBodyAtZ(t, doc, -10, -10, 10, 10, plateZ0, plateHeight)
	boss := internalBoxBodyAtZ(t, doc, -4, -3, 4, 3, bossZ0, bossHeight)
	turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 0.5}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	boss, err = Transform(t.Context(), boss, turn)
	require.NoError(t, err)
	joined, err := Union(t.Context(), plate, boss)
	require.NoError(t, err)
	fp, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.PrismUnionFillet)
	opening, err := Faces(FaceCreatedBy(CapEnd(boss))).Exactly(1).SelectFaces(joined)
	require.NoError(t, err)
	f := &unionBossShell{result: &joined.n, fp: fp, removed: faceNodes(opening), t: 1, inward: true}
	for _, source := range []*topo.Body{fp.PrismUnionFillet.First, fp.PrismUnionFillet.Second} {
		p, ok := source.Payload.(prismPayload)
		require.True(t, ok)
		rect, ok := prismshell.RecordedBossRect(p.Profile)
		require.True(t, ok)
		if rect.U1-rect.U0 > 15 {
			f.plate = source
		} else {
			f.boss = source
		}
	}
	require.NotNil(t, f.plate)
	require.NotNil(t, f.boss)
	return f
}

func (f *unionBossShell) run(t *testing.T) (*topo.Body, bool, error) {
	t.Helper()
	return prismshell.PrismUnionBossShell(t.Context(), envOf(f.result.Doc, f.result.Doc.NextProducer),
		f.result, f.fp, f.removed, f.inward, f.t, f.tDelta)
}

func editPrism(b *topo.Body, edit func(*prismPayload)) {
	p := b.Payload.(prismPayload)
	edit(&p)
	b.Payload = p
}

// TestPrismUnionBossShellGates breaks one fact of the accepted plate and
// nested boss at a time. The arm declines every broken fact without error.
func TestPrismUnionBossShellGates(t *testing.T) {
	t.Parallel()

	t.Run("accepts the nested boss", func(t *testing.T) {
		t.Parallel()
		f := unionBossShellFixture(t, 0, 4, 3, 2)
		got, matched, err := f.run(t)
		require.NoError(t, err)
		require.True(t, matched)
		require.NotNil(t, got)
	})

	shifted := func(p *prismPayload, origin r3.Vec) {
		frame, err := r3.NewFrame(origin, p.Frame.U(), p.Frame.V())
		if err != nil {
			panic(err)
		}
		p.Frame = frame
	}
	turnedFrame := func(p *prismPayload) {
		frame, err := r3.NewFrame(p.Frame.Origin(), r3.Vec{X: 0, Y: 1}, r3.Vec{X: 1})
		if err != nil {
			panic(err)
		}
		p.Frame = frame
	}
	declines := []struct {
		name string
		edit func(*unionBossShell)
	}{
		{"two removed faces", func(f *unionBossShell) { f.removed = append(f.removed, f.removed[0]) }},
		{"outward sense", func(f *unionBossShell) { f.inward = false }},
		{"held thickness", func(f *unionBossShell) { f.tDelta = 1e-9 }},
		{"zero thickness", func(f *unionBossShell) { f.t = 0 }},
		{"negative thickness", func(f *unionBossShell) { f.t = -1 }},
		{"infinite thickness", func(f *unionBossShell) { f.t = math.Inf(1) }},
		{"no fillet record", func(f *unionBossShell) { f.fp.PrismUnionFillet = nil }},
		{"placed body", func(f *unionBossShell) { f.fp.Xform, _ = r3.Translation(r3.Vec{X: 1}) }},
		{"no plate", func(f *unionBossShell) { f.fp.PrismUnionFillet = &facetedrecord.PrismUnionFilletRecord{} }},
		{"no first source", func(f *unionBossShell) {
			f.fp.PrismUnionFillet = &facetedrecord.PrismUnionFilletRecord{Second: f.fp.PrismUnionFillet.Second}
		}},
		{"no second source", func(f *unionBossShell) {
			f.fp.PrismUnionFillet = &facetedrecord.PrismUnionFilletRecord{First: f.fp.PrismUnionFillet.First}
		}},
		{"no opening", func(f *unionBossShell) { f.removed = []*topo.Face{nil} }},
		{"opening of another body", func(f *unionBossShell) {
			f.removed = []*topo.Face{topobuild.Faces(f.plate)[0]}
		}},
		{"vertex bounds short", func(f *unionBossShell) { f.fp.VertexBound = f.fp.VertexBound[:len(f.fp.VertexBound)-1] }},
		{"triangle sources short", func(f *unionBossShell) { f.fp.Src = f.fp.Src[:len(f.fp.Src)-1] }},
		{"triangle faces short", func(f *unionBossShell) { f.fp.FaceOf = f.fp.FaceOf[:len(f.fp.FaceOf)-1] }},
		{"negative volume bound", func(f *unionBossShell) { f.fp.VolSymDiff = -1 }},
		{"infinite volume bound", func(f *unionBossShell) { f.fp.VolSymDiff = math.Inf(1) }},
		{"plate is no prism", func(f *unionBossShell) { f.plate.Payload = nil }},
		{"plate section displacement", func(f *unionBossShell) { editPrism(f.plate, func(p *prismPayload) { p.SectionDelta = 1 }) }},
		{"plate bottom displacement", func(f *unionBossShell) { editPrism(f.plate, func(p *prismPayload) { p.Z0Delta = 1 }) }},
		{"plate top displacement", func(f *unionBossShell) { editPrism(f.plate, func(p *prismPayload) { p.Z1Delta = 1 }) }},
		{"plate surface result", func(f *unionBossShell) { editPrism(f.plate, func(p *prismPayload) { p.SurfaceResult = true }) }},
		{"plate placed", func(f *unionBossShell) {
			editPrism(f.plate, func(p *prismPayload) { p.Xform, _ = r3.Translation(r3.Vec{X: 1}) })
		}},
		{"plate frame turned", func(f *unionBossShell) { editPrism(f.plate, turnedFrame) }},
		{"plate off the origin in x", func(f *unionBossShell) {
			editPrism(f.plate, func(p *prismPayload) { shifted(p, r3.Vec{X: 1}) })
		}},
		{"plate off the origin in y", func(f *unionBossShell) {
			editPrism(f.plate, func(p *prismPayload) { shifted(p, r3.Vec{Y: 1}) })
		}},
		{"plate profile with a hole", func(f *unionBossShell) {
			editPrism(f.plate, func(p *prismPayload) { p.Profile.Holes = append(p.Profile.Holes, p.Profile.Outer) })
		}},
		{"boss is no prism", func(f *unionBossShell) { f.boss.Payload = nil }},
		{"boss section displacement", func(f *unionBossShell) { editPrism(f.boss, func(p *prismPayload) { p.SectionDelta = 1 }) }},
		{"boss bottom displacement", func(f *unionBossShell) { editPrism(f.boss, func(p *prismPayload) { p.Z0Delta = 1 }) }},
		{"boss top displacement", func(f *unionBossShell) { editPrism(f.boss, func(p *prismPayload) { p.Z1Delta = 1 }) }},
		{"boss surface result", func(f *unionBossShell) { editPrism(f.boss, func(p *prismPayload) { p.SurfaceResult = true }) }},
		{"boss frame turned", func(f *unionBossShell) { editPrism(f.boss, turnedFrame) }},
		{"boss off the origin in x", func(f *unionBossShell) {
			editPrism(f.boss, func(p *prismPayload) { shifted(p, r3.Vec{X: 1}) })
		}},
		{"boss off the origin in y", func(f *unionBossShell) {
			editPrism(f.boss, func(p *prismPayload) { shifted(p, r3.Vec{Y: 1}) })
		}},
		{"boss profile with a hole", func(f *unionBossShell) {
			editPrism(f.boss, func(p *prismPayload) { p.Profile.Holes = append(p.Profile.Holes, p.Profile.Outer) })
		}},
		{"boss tilted", func(f *unionBossShell) {
			editPrism(f.boss, func(p *prismPayload) {
				tilt, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(10))
				if err != nil {
					panic(err)
				}
				p.Xform = tilt
			})
		}},
		{"boss lifted", func(f *unionBossShell) {
			editPrism(f.boss, func(p *prismPayload) {
				lift, err := p.Xform.Then(mustTranslation(r3.Vec{Z: 1}))
				if err != nil {
					panic(err)
				}
				p.Xform = lift
			})
		}},
		{"thickness not exact", func(f *unionBossShell) { f.t = 0.1 }},
		{"thickness fills the boss", func(f *unionBossShell) { f.t = 3 }},
	}
	for _, tc := range declines {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			f := unionBossShellFixture(t, 0, 4, 3, 2)
			tc.edit(f)
			_, matched, err := f.run(t)
			require.NoError(t, err)
			require.False(t, matched)
		})
	}

	// Heights whose float sums round decline for the exactness of the four
	// level readings.
	for _, tc := range []struct {
		name                                     string
		plateZ0, plateHeight, bossZ0, bossHeight float64
	}{
		{"interface level", 0.1, 0.2, 0.2, 2},
		{"boss top level", 0, 4, 3.1, 2},
	} {
		t.Run("inexact "+tc.name, func(t *testing.T) {
			t.Parallel()
			f := unionBossShellFixture(t, tc.plateZ0, tc.plateHeight, tc.bossZ0, tc.bossHeight)
			_, matched, err := f.run(t)
			require.NoError(t, err)
			require.False(t, matched)
		})
	}
}

func mustTranslation(v r3.Vec) r3.Transform {
	m, err := r3.Translation(v)
	if err != nil {
		panic(err)
	}
	return m
}
