package brepwallmesh

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// WallMesh is one swept face's chording: its rim polylines at z0 and z1 as
// mesh vertex indices in walk order (closed for a whole circle), its largest
// sagitta, and the chord-versus-arc charges tessellation §5 reads.
type WallMesh struct {
	// Cols holds one vertex column per rim sample, ascending from z0 to z1,
	// and Levels each column's heights: a side line's column stops at its
	// own splits alone, so the neighbouring face across it meets exactly the
	// side pieces' vertices, and every interior column at every split of
	// either side. Bottom and Top are the rims, each column's first and last
	// vertex.
	Cols        [][]int
	Levels      [][]float64
	Bottom, Top []int
	Sag         float64
	WallSlack   float64
	CapSlack    float64
	SegmentArea float64
}

// ChordWall chords one swept face's wall with tessellation.SampleLoop
// over its one walk and places its rim samples at both levels. An open wall's
// polyline ends at its walk's end; a whole circle's closes on its first
// sample. Each end sample is charged the bound from its held point to the
// point the wall's record denotes there (boundarywalk.DenotedStartBound and
// DenotedEndBound), which adds an arc's radial residual at its natural t = 1
// end; addVertex keeps the largest bound any use places at one vertex.
// imposed, when non-nil, replaces the chording with the samples a route L band
// states for this wall (tessellation.ImposeBandWall); the wall's own end is still appended.
func ChordWall[F any](ctx context.Context, f breprecord.Face, w survey2d.SegmentWalk, e brepgeom.Embed, face F, chord float64,
	work *freeform.FreeformWork, imposed *tessellation.ChordSamples[F], addVertex func([3]float64, proofbound.WalkEndBound) int) (WallMesh, error) {
	walk := survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}
	var sampled tessellation.ChordSamples[F]
	if imposed != nil {
		sampled = *imposed
	} else {
		var err error
		sampled, err = tessellation.SampleLoop[F]([]survey2d.SideWalk{walk}, []sectionrecord.CurveSegment{f.Wall}, chord,
			f.Z1-f.Z0, work, proofbound.NewWorkBudget(ctx), func(survey2d.SideWalk) (F, error) { return face, nil },
			stationbound.ChordStationBound)
		if err != nil {
			return WallMesh{}, err
		}
	}
	samples, bounds := sampled.Samples, sampled.BoundOf
	if !w.Closed {
		samples = append(samples, sectionrecord.Point2{U: w.EndU, V: w.EndV})
		bounds = append(bounds, boundarywalk.DenotedEndBound(f.Wall, w))
	}
	wm := WallMesh{Sag: sampled.MaxSag, WallSlack: sampled.WallSlack, CapSlack: sampled.CapSlack,
		SegmentArea: sampled.SegmentArea}
	// Each side line's column holds a vertex at its own splits alone, so the
	// side pieces' vertices are the wall's own and the face across the side
	// line meets no vertex it does not hold; the interior columns hold every
	// split of either side, and tessellation.ZipColumns closes the strips between.
	levelsOf := func(splits ...[]brepgeom.Split) []float64 {
		out := []float64{f.Z0}
		for _, side := range splits {
			for _, sp := range side {
				out = append(out, sp.Z)
			}
		}
		out = append(out, f.Z1)
		slices.Sort(out)
		return slices.Compact(out)
	}
	every := levelsOf(f.Side0, f.Side1)
	for j, p := range samples {
		levels := every
		switch {
		case w.Closed:
		case j == 0:
			levels = levelsOf(f.Side0)
		case j == len(samples)-1:
			levels = levelsOf(f.Side1)
		}
		col := make([]int, 0, len(levels))
		for _, z := range levels {
			col = append(col, addVertex(e.Canon(p.U, p.V, z), bounds[j]))
		}
		wm.Cols = append(wm.Cols, col)
		wm.Levels = append(wm.Levels, levels)
		wm.Bottom = append(wm.Bottom, col[0])
		wm.Top = append(wm.Top, col[len(col)-1])
	}
	if w.Closed {
		wm.Cols = append(wm.Cols, wm.Cols[0])
		wm.Levels = append(wm.Levels, wm.Levels[0])
		wm.Bottom = append(wm.Bottom, wm.Bottom[0])
		wm.Top = append(wm.Top, wm.Top[0])
	}
	return wm, nil
}

// WallClearance refuses a mesh whose chorded walls could cross where the
// analytic walls do not: every two walls sweeping along one reference axis over
// overlapping intervals, and sharing no endpoint, must clear each other by
// more than their summed sagittae plus the section's rounding floor — the
// per-slab loop clearance the stacked tessellator proves, read per wall pair.
// Each open wall polyline is walked out and back so the loop clearance reader
// measures exactly its own chords.
func WallClearance(ctx context.Context, bp breprecord.Payload, btopo *breprecord.Topology, walls map[int]WallMesh, canon [][3]float64) error {
	type wallLine struct {
		axis   int
		lo, hi float64
		pts    []sectionrecord.Point2
		sag    float64
	}
	var lines []wallLine
	for fi, f := range bp.Faces {
		wm, ok := walls[fi]
		if !ok {
			continue
		}
		e := btopo.Embeds[fi]
		k := e.Axis[2]
		lo, hi := e.Sign[2]*f.Z0, e.Sign[2]*f.Z1
		if lo > hi {
			lo, hi = hi, lo
		}
		line := wallLine{axis: k, lo: lo, hi: hi, sag: wm.Sag}
		for _, vi := range wm.Bottom {
			c := canon[vi]
			var p [2]float64
			n := 0
			for i, x := range c {
				if i != k {
					p[n] = x
					n++
				}
			}
			line.pts = append(line.pts, sectionrecord.Point2{U: p[0], V: p[1]})
		}
		lines = append(lines, line)
	}
	for i := range lines {
		for j := i + 1; j < len(lines); j++ {
			if err := ctx.Err(); err != nil {
				return err
			}
			a, b := lines[i], lines[j]
			if a.axis != b.axis || math.Max(a.lo, b.lo) >= math.Min(a.hi, b.hi) {
				continue
			}
			ends := []sectionrecord.Point2{a.pts[0], a.pts[len(a.pts)-1]}
			if slices.Contains(ends, b.pts[0]) || slices.Contains(ends, b.pts[len(b.pts)-1]) {
				continue
			}
			pts := append(slices.Clone(a.pts), b.pts...)
			if err := tessellation.RequireLoopClearance(ctx, pts,
				[][]int{tessellation.OutAndBack(0, len(a.pts)), tessellation.OutAndBack(len(a.pts), len(b.pts))},
				[]float64{a.sag, b.sag}); err != nil {
				return err
			}
		}
	}
	return nil
}
