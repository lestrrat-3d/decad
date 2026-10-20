package loftrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMirrorCapTrianglesMatch(t *testing.T) {
	t.Parallel()

	lower := loftrecord.Payload{Walls: 1, CapStartCount: 1,
		Tris: [][3]int{{9, 9, 9}, {8, 8, 8}, {0, 1, 2}, {2, 1, 3}}}
	mirrored := func(tris ...[3]int) loftrecord.Payload {
		return loftrecord.Payload{Tris: append([][3]int{{7, 7, 7}, {6, 6, 6}}, tris...)}
	}

	require.True(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1}, [3]int{3, 1, 2})), "reversed winding")
	require.True(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{3, 1, 2}, [3]int{0, 2, 1})), "any order")
	require.True(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{2, 1, 0}, [3]int{1, 2, 3})), "rotated corners")
	require.False(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 1, 2}, [3]int{3, 1, 2})), "same winding")
	require.False(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1})), "a triangle is missing")
	require.False(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1}, [3]int{3, 1, 2}, [3]int{0, 3, 1})),
		"a triangle is extra")
	require.False(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1}, [3]int{3, 1, 4})), "different triangle")
	require.False(t, lower.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1}, [3]int{0, 2, 1})), "a triangle repeats")

	twice := loftrecord.Payload{Walls: 1, CapStartCount: 1,
		Tris: [][3]int{{9, 9, 9}, {8, 8, 8}, {0, 1, 2}, {1, 2, 0}}}
	require.False(t, twice.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1}, [3]int{0, 2, 1})),
		"the lower loft uses one triangle twice")
	require.False(t, twice.MirrorCapTrianglesMatch(mirrored([3]int{0, 2, 1})),
		"the lower loft uses one triangle twice and the upper once")
}

func TestMirrorAxialSource(t *testing.T) {
	t.Parallel()

	frame := func(origin r3.Vec, u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		return f
	}
	x, y := r3.Vec{X: 1}, r3.Vec{Y: 1}
	base := loftrecord.Payload{Frame0: frame(r3.Vec{}, x, y), Frame1: frame(r3.Vec{Z: 5}, x, y)}
	require.True(t, base.MirrorAxialSource())

	for name, edit := range map[string]func(*loftrecord.Payload){
		"start off the axis in x": func(p *loftrecord.Payload) { p.Frame0 = frame(r3.Vec{X: 1}, x, y) },
		"start off the axis in y": func(p *loftrecord.Payload) { p.Frame0 = frame(r3.Vec{Y: 1}, x, y) },
		"end off the axis in x":   func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{X: 1, Z: 5}, x, y) },
		"end off the axis in y":   func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{Y: 1, Z: 5}, x, y) },
		"start u along z":         func(p *loftrecord.Payload) { p.Frame0 = frame(r3.Vec{}, r3.Vec{Z: 1}, y) },
		"start turned":            func(p *loftrecord.Payload) { p.Frame0 = frame(r3.Vec{}, y, r3.Vec{X: -1}) },
		"end turned":              func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{Z: 5}, y, r3.Vec{X: -1}) },
		"start v flipped":         func(p *loftrecord.Payload) { p.Frame0 = frame(r3.Vec{}, x, y.Scale(-1)) },
		"end v flipped":           func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{Z: 5}, x, y.Scale(-1)) },
		"end below start":         func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{Z: -5}, x, y) },
		"end level with start":    func(p *loftrecord.Payload) { p.Frame1 = frame(r3.Vec{}, x, y) },
	} {
		p := base
		edit(&p)
		require.False(t, p.MirrorAxialSource(), name)
	}
}
