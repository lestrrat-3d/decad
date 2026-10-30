package apitest_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

type circularPatternBenchFixture struct {
	cubes  *cubes
	center r3.Vec
	want   float64
}

// BenchmarkCircularPatternCubeSet measures the public faceted-solid pattern
// route with a deterministic cube set accepted by the differential test.
func BenchmarkCircularPatternCubeSet(b *testing.B) {
	fixture := circularPatternFixture(b)
	b.Logf("fixture seed=31,32; cubes=%v; center=%v; count=4; oracle-volume=%.0f",
		circularPatternCubeCells(fixture.cubes), fixture.center, fixture.want)
	// SolidFromMesh yields a faceted payload with zero vertex bounds. Count four
	// about +Z makes instances one through three exact quarter turns.
	spec := decad.CircularPattern{Center: fixture.center, Axis: r3.NewVec(0, 0, 1), Count: 4}
	b.ReportAllocs()

	b.Run("EndToEnd", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			doc := decad.New()
			source := circularPatternMeshSolid(b, doc, fixture.cubes)
			out, err := decad.Patterned(b.Context(), source, spec)
			if err != nil {
				b.Fatal(err)
			}
			checkCircularPatternVolume(b, out, fixture.want)
		}
	})

	b.Run("PatternedOnly", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			b.StopTimer()
			doc := decad.New()
			source := circularPatternMeshSolid(b, doc, fixture.cubes)
			b.StartTimer()
			out, err := decad.Patterned(b.Context(), source, spec)
			if err != nil {
				b.Fatal(err)
			}
			b.StopTimer()
			checkCircularPatternVolume(b, out, fixture.want)
			b.StartTimer()
		}
	})
}

func circularPatternFixture(b *testing.B) circularPatternBenchFixture {
	b.Helper()
	rng := rand.New(rand.NewPCG(31, 32))
	for range 40 {
		c := randomCubes(rng)
		doc := decad.New()
		source := circularPatternMeshSolidOrNil(b, doc, c)
		count := []int{2, 4}[rng.IntN(2)]
		center := r3.NewVec(float64(rng.IntN(voxelSide+1)), float64(rng.IntN(voxelSide+1)), 0)
		if source == nil {
			continue
		}
		out, err := decad.Patterned(b.Context(), source, decad.CircularPattern{
			Center: center, Axis: r3.NewVec(0, 0, 1), Count: count,
		})
		if err != nil || count != 4 {
			continue
		}
		want := circularPatternCoveredCubeCount(c, center)
		checkCircularPatternVolume(b, out, float64(want))
		return circularPatternBenchFixture{cubes: c, center: center, want: float64(want)}
	}
	b.Fatal("could not select an accepted count-four fixture from the differential generator")
	return circularPatternBenchFixture{}
}

func circularPatternMeshSolid(b *testing.B, doc *decad.Document, c *cubes) *decad.Body {
	b.Helper()
	body := circularPatternMeshSolidOrNil(b, doc, c)
	if body == nil {
		b.Fatal("selected cube set was not admitted as a mesh solid")
	}
	return body
}

func circularPatternMeshSolidOrNil(b *testing.B, doc *decad.Document, c *cubes) *decad.Body {
	b.Helper()
	verts, tris, ok := c.mesh()
	if !ok {
		return nil
	}
	mesh, err := decad.NewMeshBody(b.Context(), verts, tris)
	if err != nil {
		b.Fatal(err)
	}
	body, err := decad.SolidFromMesh(b.Context(), doc, mesh)
	if err != nil {
		return nil
	}
	return body
}

func circularPatternCoveredCubeCount(c *cubes, center r3.Vec) int {
	covered := map[[3]int]struct{}{}
	for x := range voxelSide {
		for y := range voxelSide {
			for z := range voxelSide {
				if !c[x][y][z] {
					continue
				}
				dx, dy := float64(x)+0.5-center.X, float64(y)+0.5-center.Y
				for range 4 {
					covered[[3]int{int(math.Floor(center.X + dx)), int(math.Floor(center.Y + dy)), z}] = struct{}{}
					dx, dy = -dy, dx
				}
			}
		}
	}
	return len(covered)
}

func circularPatternCubeCells(c *cubes) [][3]int {
	cells := make([][3]int, 0, c.count())
	for x := range voxelSide {
		for y := range voxelSide {
			for z := range voxelSide {
				if c[x][y][z] {
					cells = append(cells, [3]int{x, y, z})
				}
			}
		}
	}
	return cells
}

func checkCircularPatternVolume(b *testing.B, body *decad.Body, want float64) {
	b.Helper()
	volume, err := body.Volume()
	if err != nil {
		b.Fatal(err)
	}
	got, bound := volume.Value.Base(), volume.Bound.Base()
	if math.Abs(got-want) > bound+1e-9 {
		b.Fatalf("pattern volume %.12g ± %.3g, want %.12g", got, bound, want)
	}
}
