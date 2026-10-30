package boolupstream_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boolupstream"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// rectangle is the closed loop u0..u1 by v0..v1.
func rectangle(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// prism builds the rectangle u0..u1 by v0..v1 in the frame spanned by u and v,
// swept along u x v from z0 to z1, under producer ref.
func prism(t *testing.T, ref provenance.Producer, u, v r3.Vec, u0, v0, u1, v1, z0, z1 float64) *topo.Body {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, u, v)
	require.NoError(t, err)
	pp := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: rectangle(u0, v0, u1, v1)},
		Frame:   frame, Z0: z0, Z1: z1, Xform: r3.Identity(),
	}
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: ref, Alloc: topotest.Alloc{}}, pp,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	return body
}

var ex, ey, ez = r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

// plateAndBoss returns a plate 80 x 60 from z = 0 to 10 (producer 1), a boss
// 20 x 20 on it up to z = 25 (producer 2), and a copy of the plate under
// producer 3 that stands in for the result: its caps lie at the plate's levels.
func plateAndBoss(t *testing.T) (*topo.Body, *topo.Body, *topo.Body) {
	t.Helper()
	plate := prism(t, 1, ex, ey, 0, 0, 80, 60, 0, 10)
	boss := prism(t, 2, ex, ey, 30, 20, 50, 40, 10, 25)
	result := prism(t, 3, ex, ey, 0, 0, 80, 60, 0, 10)
	return plate, boss, result
}

func capOf(t *testing.T, b *topo.Body, role string) *topo.Face {
	t.Helper()
	for _, f := range topobuild.Faces(b) {
		if len(f.Origins) == 1 && f.Origins[0].Role == role {
			return f
		}
	}
	require.FailNow(t, "no face has role "+role)
	return nil
}

// attributed is the table row of the result face whose own role is role, nil
// when the table has none.
func attributed(table topo.UpstreamRoles, role string) []provenance.Ref {
	for _, e := range table {
		if len(e.Roles) == 1 && e.Roles[0] == role {
			return e.Origins
		}
	}
	return nil
}

func TestAttribute(t *testing.T) {
	t.Parallel()
	plateStart := []provenance.Ref{{Producer: 1, Role: "capStart"}}
	plateEnd := []provenance.Ref{{Producer: 1, Role: "capEnd"}}
	t.Run("plate caps", func(t *testing.T) {
		t.Parallel()
		plate, boss, result := plateAndBoss(t)
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, boss)
		require.NoError(t, err)
		require.Len(t, table, 2)
		require.Equal(t, plateStart, attributed(table, "capStart"))
		// The boss's start cap shares z = 10 but faces down, so only the
		// plate's end cap bounds an up-facing face there.
		require.Equal(t, plateEnd, attributed(table, "capEnd"))
	})
	t.Run("denoted level", func(t *testing.T) {
		t.Parallel()
		plate, boss, result := plateAndBoss(t)
		start := capOf(t, boss, "capStart")
		start.AxialDelta = 1e-9
		start.CapRecord = &topo.CapRecord{Frame: start.CapRecord.Frame, Xform: start.CapRecord.Xform,
			Z: start.CapRecord.Z, ZDenoted: big.NewRat(10, 1)}
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, boss)
		require.NoError(t, err)
		require.Equal(t, plateEnd, attributed(table, "capEnd"))
	})
	// Each case leaves the boss face at z = 10 with no exact plane, so no
	// face of the result on a plane parallel to it can be placed.
	for name, mutate := range map[string]func(boss *topo.Body){
		"faceted tag":  func(boss *topo.Body) { capOf(t, boss, "capStart").Surface = surfacegeom.Faceted{} },
		"retained tag": func(boss *topo.Body) { capOf(t, boss, "capStart").RetainedTag = true },
		"displaced level": func(boss *topo.Body) {
			f := capOf(t, boss, "capStart")
			f.AxialDelta = 1e-9
			f.CapRecord = &topo.CapRecord{Frame: f.CapRecord.Frame, Xform: f.CapRecord.Xform, Z: f.CapRecord.Z}
		},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			plate, boss, result := plateAndBoss(t)
			mutate(boss)
			table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, boss)
			require.NoError(t, err)
			require.Empty(t, table)
		})
	}
	t.Run("second candidate", func(t *testing.T) {
		t.Parallel()
		plate, boss, result := plateAndBoss(t)
		// The boss start cap turned to face up is a second up-facing face on
		// z = 10, so the result's face there names neither; the bottom cap,
		// which no boss face reaches, still names the plate's.
		f := capOf(t, boss, "capStart")
		tag := f.Surface.(surfacegeom.Plane)
		frame, err := r3.NewFrame(tag.Frame.Origin(), tag.Frame.V(), tag.Frame.U())
		require.NoError(t, err)
		f.Surface = surfacegeom.Plane{Frame: frame}
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, boss)
		require.NoError(t, err)
		require.Nil(t, attributed(table, "capEnd"))
		require.Equal(t, plateStart, attributed(table, "capStart"))
	})
	t.Run("wall parallel to the plane", func(t *testing.T) {
		t.Parallel()
		plate, _, result := plateAndBoss(t)
		// A bar swept along x whose top wall lies on z = 10: the result's
		// face there may hold points of that wall.
		bar := prism(t, 2, ey, ez, 0, 5, 10, 10, 0, 100)
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, bar)
		require.NoError(t, err)
		require.Empty(t, table)
	})
	t.Run("one role two answers", func(t *testing.T) {
		t.Parallel()
		plate, boss, result := plateAndBoss(t)
		capOf(t, result, "capStart").Origins[0].Role = "capEnd"
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpUnion, result, plate, boss)
		require.NoError(t, err)
		require.Empty(t, table)
	})
	t.Run("cut tool", func(t *testing.T) {
		t.Parallel()
		plate, _, _ := plateAndBoss(t)
		// A tool from z = 5 to 15 leaves a pocket whose up-facing floor at
		// z = 5 lies inside the tool's down-facing start cap.
		tool := prism(t, 2, ex, ey, 10, 10, 20, 20, 5, 15)
		floor := prism(t, 3, ex, ey, 10, 10, 20, 20, 0, 5)
		table, err := boolupstream.Attribute(t.Context(), meshbool.OpCut, floor, plate, tool)
		require.NoError(t, err)
		require.Equal(t, []provenance.Ref{{Producer: 2, Role: "capStart"}}, attributed(table, "capEnd"))
		require.Equal(t, plateStart, attributed(table, "capStart"))
		table, err = boolupstream.Attribute(t.Context(), meshbool.OpUnion, floor, plate, tool)
		require.NoError(t, err)
		require.Nil(t, attributed(table, "capEnd"))
	})
}

// TestAttach records the table on a prism result's payload, and a placed
// rebuild of that payload re-attaches it under the new producer.
func TestAttach(t *testing.T) {
	t.Parallel()
	plate, boss, result := plateAndBoss(t)
	require.NoError(t, boolupstream.Attach(t.Context(), meshbool.OpUnion, result, plate, boss))
	require.Equal(t, []provenance.Ref{{Producer: 1, Role: "capEnd"}}, capOf(t, result, "capEnd").Upstream)
	pp, ok := result.Payload.(prismrecord.Payload)
	require.True(t, ok)
	require.Len(t, pp.Upstream, 2)

	move, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	placed, err := pp.Placed(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}, move)
	require.NoError(t, err)
	require.Equal(t, []provenance.Ref{{Producer: 1, Role: "capEnd"}}, capOf(t, placed, "capEnd").Upstream)
	require.Equal(t, provenance.Producer(9), capOf(t, placed, "capEnd").Origins[0].Producer)

	// A record derived by copying the payload and built afresh carries none.
	derived, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 10, Alloc: topotest.Alloc{}}, pp,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Nil(t, capOf(t, derived, "capEnd").Upstream)
	require.Nil(t, derived.Payload.(prismrecord.Payload).Upstream)
}
