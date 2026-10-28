package apitest_test

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"hash"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sectorDigestEnv names the directory the TestSectorRouteDigest tests write
// their per-fixture digests to, one file per test. A change that must leave the
// +X sector route bit-identical runs these tests on its base commit and on its
// head, on one machine, and compares the two directories byte for byte
// (agent-docs/revolve-axis-design.md §8.2). Each commit runs its own copy of
// these tests. The tests skip unless the variable names a directory, so CI does
// not run them, and a pull request that needs the comparison sets it for both
// the base and the head run.
const sectorDigestEnv = "DECAD_SECTOR_DIGEST"

// sectorDigestTolerance is the VerifyAll tolerance every fixture tessellates at.
const sectorDigestTolerance = 0.1

type sectorDigestFixture struct {
	name  string
	build func(t *testing.T) []*decad.Body
}

// runSectorDigest hashes the public readings of every fixture of one group on
// the finite Revolve and point-Loft sector route: each tessellated vertex, each
// body vertex with its bound, the volume, area and centroid with their bounds,
// and the bounding box. The readings carry the payload's mesh bound,
// symmetric-difference volume and area slack, so no unexported state is read.
// The first fixture of each group is built twice and must hash equally. The
// groups are separate tests so the CI shards can place them independently.
// It skips the group unless sectorDigestEnv names a directory.
func runSectorDigest(t *testing.T, group string, fixtures []sectorDigestFixture) {
	dir := os.Getenv(sectorDigestEnv)
	if dir == "" {
		t.Skipf("%s names no directory; set it to compare base and head digests (agent-docs/revolve-axis-design.md §8.2)",
			sectorDigestEnv)
	}
	var report strings.Builder
	for i, fixture := range fixtures {
		t.Run(fixture.name, func(t *testing.T) {
			digest := sectorDigest(t, fixture.build(t))
			if i == 0 {
				require.Equal(t, digest, sectorDigest(t, fixture.build(t)),
					"two builds of one fixture hash differently")
			}
			fmt.Fprintf(&report, "%s %s\n", fixture.name, digest)
		})
	}
	if !t.Failed() {
		require.NoError(t, os.MkdirAll(dir, 0o700))
		require.NoError(t, os.WriteFile(filepath.Join(dir, group+".txt"), []byte(report.String()), 0o600))
	}
}

func sectorDigest(t *testing.T, bodies []*decad.Body) string {
	t.Helper()
	h := sha256.New()
	for _, body := range bodies {
		digestBody(t, h, body)
	}
	return hex.EncodeToString(h.Sum(nil))
}

func digestFloats(h hash.Hash, values ...float64) {
	var buf [8]byte
	for _, v := range values {
		binary.LittleEndian.PutUint64(buf[:], math.Float64bits(v))
		h.Write(buf[:])
	}
}

func digestVec(h hash.Hash, v r3.Vec) { digestFloats(h, v.X, v.Y, v.Z) }

func digestBody(t *testing.T, h hash.Hash, body *decad.Body) {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(sectorDigestTolerance),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, v := range mesh.Vertices() {
		digestVec(h, v)
	}
	for _, v := range body.Vertices() {
		p := v.Position()
		digestVec(h, p.Value)
		digestFloats(h, p.Bound.Base())
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	digestFloats(h, volume.Value.Base(), volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	digestFloats(h, area.Value.Base(), area.Bound.Base())
	centroid, err := body.Centroid()
	require.NoError(t, err)
	digestVec(h, centroid.Value)
	digestFloats(h, centroid.Bound.Base())
	box, err := body.Bounds()
	require.NoError(t, err)
	digestVec(h, box.Min)
	digestVec(h, box.Max)
	digestFloats(h, box.Bound.Base())
}

func sectorDigestTurn(t *testing.T, radians float64) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(radians))
	require.NoError(t, err)
	return turn
}

func sectorDigestCopy(t *testing.T, body *decad.Body, degrees float64) *decad.Body {
	t.Helper()
	placed, err := decad.Copy(t.Context(), body,
		decad.WithCopyPlacement(sectorDigestTurn(t, degrees*math.Pi/180)))
	require.NoError(t, err)
	return placed
}

func sectorDigestUnion(t *testing.T, first *decad.Body, rest ...*decad.Body) *decad.Body {
	t.Helper()
	joined, err := decad.Union(t.Context(), first, rest...)
	require.NoError(t, err)
	return joined
}

// sectorDigestSixSpanHub is the six-span hub of TestUnionSixSpanHubSerialRadialLugs.
func sectorDigestSixSpanHub(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: 9, V: 3},
		{U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = section.CreatePoint(point.U, point.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	hub, err := decad.Revolve(t.Context(), doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return hub
}

func sectorDigestCylinderCut(t *testing.T, radius float64, copies int) []*decad.Body {
	t.Helper()
	doc, hub, lug, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	var joined *decad.Body
	if copies > 1 {
		joined = sectorDigestUnion(t, hub, lug, sectorDigestCopy(t, lug, 180))
	} else {
		joined = sectorDigestUnion(t, hub, lug)
	}
	tool := revolveSectorCylinderTool(t, doc, radius, 4, 6, 0)
	cut, err := decad.Cut(t.Context(), joined, tool)
	require.NoError(t, err)
	return []*decad.Body{cut}
}

func sectorDigestPointCone(t *testing.T, apex float64, cut, ordered bool) []*decad.Body {
	t.Helper()
	doc := decad.New()
	tooth := pointConeTooth(t, doc)
	if ordered {
		var err error
		tooth, err = decad.Cut(t.Context(), tooth, pointConeTool(t, doc, 5))
		require.NoError(t, err)
	}
	tool := pointConeTool(t, doc, apex)
	var got *decad.Body
	var err error
	if cut {
		got, err = decad.Cut(t.Context(), tooth, tool)
	} else {
		got, err = decad.Intersect(t.Context(), tooth, tool)
	}
	require.NoError(t, err)
	return []*decad.Body{got}
}

func sectorDigestTapered(splitU float64, fitted bool) func(t *testing.T) []*decad.Body {
	return func(t *testing.T) []*decad.Body {
		_, hub, trimmed, _, _ := taperedHubAndRadialLug(t, splitU, 7.5, 4.3, fitted)
		return []*decad.Body{sectorDigestUnion(t, hub, trimmed)}
	}
}

func sectorDigestNearAxis(serial bool) func(t *testing.T) []*decad.Body {
	return func(t *testing.T) []*decad.Body {
		_, hub, trimmed := shoulderedHubAndRadialLug(t)
		near, err := decad.Copy(t.Context(), trimmed,
			decad.WithCopyPlacement(sectorDigestTurn(t, 2*math.Pi*5/16)))
		require.NoError(t, err)
		if serial {
			return []*decad.Body{sectorDigestUnion(t, sectorDigestUnion(t, hub, trimmed), near)}
		}
		return []*decad.Body{sectorDigestUnion(t, hub, trimmed, near)}
	}
}

func sectorDigestPointConeFixture(apex float64, cut, ordered bool) func(t *testing.T) []*decad.Body {
	return func(t *testing.T) []*decad.Body { return sectorDigestPointCone(t, apex, cut, ordered) }
}

func sectorDigestCylinderFixture(radius float64, copies int) func(t *testing.T) []*decad.Body {
	return func(t *testing.T) []*decad.Body { return sectorDigestCylinderCut(t, radius, copies) }
}

// TestSectorRouteDigestSingleLug covers one lug joined to each hub shape.
func TestSectorRouteDigestSingleLug(t *testing.T) {
	t.Parallel()
	runSectorDigest(t, "single-lug", []sectorDigestFixture{
		{"TaperedHubAndRadialLug", sectorDigestTapered(9, false)},
		{"TaperedHubAndFittedRadialLug", sectorDigestTapered(9, true)},
		{"ShoulderedHubAndRadialLug", func(t *testing.T) []*decad.Body {
			_, hub, trimmed := shoulderedHubAndRadialLug(t)
			return []*decad.Body{sectorDigestUnion(t, hub, trimmed)}
		}},
		{"SlopedHubCapAndRadialLug", sectorDigestTapered(8.99, false)},
	})
}

// TestSectorRouteDigestCopiedLugs covers serial NextSectorUnion and variadic
// SectorUnionMany over rotated copies of one lug.
func TestSectorRouteDigestCopiedLugs(t *testing.T) {
	t.Parallel()
	runSectorDigest(t, "copied-lugs", []sectorDigestFixture{
		{"ShoulderedHubAndTwoRadialLugsSerial", func(t *testing.T) []*decad.Body {
			_, hub, trimmed := shoulderedHubAndRadialLug(t)
			second := sectorDigestCopy(t, trimmed, 180)
			return []*decad.Body{sectorDigestUnion(t, sectorDigestUnion(t, hub, trimmed), second)}
		}},
		{"SixSpanHubSerialRadialLugs", func(t *testing.T) []*decad.Body {
			doc, _, trimmed, _, _ := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
			hub := sectorDigestSixSpanHub(t, doc)
			placed := []*decad.Body{sectorDigestCopy(t, trimmed, 120), sectorDigestCopy(t, trimmed, 240)}
			joined := sectorDigestUnion(t, hub, trimmed)
			for _, p := range placed {
				joined = sectorDigestUnion(t, joined, p)
			}
			return []*decad.Body{joined}
		}},
		{"ShoulderedHubAndTwoRadialLugsVariadic", func(t *testing.T) []*decad.Body {
			_, hub, trimmed := shoulderedHubAndRadialLug(t)
			return []*decad.Body{sectorDigestUnion(t, hub, trimmed, sectorDigestCopy(t, trimmed, 180))}
		}},
		{"ShoulderedHubAndThreeRadialLugsVariadic", func(t *testing.T) []*decad.Body {
			_, hub, trimmed := shoulderedHubAndRadialLug(t)
			return []*decad.Body{sectorDigestUnion(t, hub, trimmed,
				sectorDigestCopy(t, trimmed, 120), sectorDigestCopy(t, trimmed, 240))}
		}},
		{"ShoulderedHubWithNearAxisCopySerial", sectorDigestNearAxis(true)},
		{"ShoulderedHubWithNearAxisCopyVariadic", sectorDigestNearAxis(false)},
	})
}

// TestSectorRouteDigestManyLugs joins 40 copies of one lug in one Union.
func TestSectorRouteDigestManyLugs(t *testing.T) {
	t.Parallel()
	if testing.Short() {
		t.Skip("joins 40 sectors")
	}
	runSectorDigest(t, "many-lugs", []sectorDigestFixture{
		{"HubAndManyRadialLugs", func(t *testing.T) []*decad.Body {
			_, hub, lug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.1, false)
			const count = 40
			rest := []*decad.Body{lug}
			for i := 1; i < count; i++ {
				copied, err := decad.Copy(t.Context(), lug,
					decad.WithCopyPlacement(sectorDigestTurn(t, 2*math.Pi*float64(i)/count)))
				require.NoError(t, err)
				rest = append(rest, copied)
			}
			return []*decad.Body{sectorDigestUnion(t, hub, rest...)}
		}},
	})
}

// TestSectorRouteDigestBevel covers the circle-fragment and partial-fit root
// routes of the embedded-root tooth joins.
func TestSectorRouteDigestBevel(t *testing.T) {
	t.Parallel()
	if testing.Short() {
		t.Skip("joins five sectors")
	}
	runSectorDigest(t, "bevel", []sectorDigestFixture{
		{"BevelBlankAndEmbeddedRootTeeth", func(t *testing.T) []*decad.Body {
			_, joined, err := bevel32.join(t, 3)
			require.NoError(t, err)
			return []*decad.Body{joined}
		}},
		{"BevelTipTouchingHeelConeMatchingFaces", func(t *testing.T) []*decad.Body {
			_, joined, err := bevel16.join(t, 2, decad.WithMatchingFaces())
			require.NoError(t, err)
			return []*decad.Body{joined}
		}},
	})
}

// TestSectorRouteDigestCylinderCut covers the coaxial cylinder Cut of a joined
// sector body.
func TestSectorRouteDigestCylinderCut(t *testing.T) {
	t.Parallel()
	runSectorDigest(t, "cylinder-cut", []sectorDigestFixture{
		{"AnnularCylinderCut", sectorDigestCylinderFixture(1, 1)},
		{"RepeatedAnnularCylinderCut", sectorDigestCylinderFixture(1, 2)},
		{"StraightBandCylinderCut", sectorDigestCylinderFixture(2, 1)},
		{"RepeatedStraightBandCylinderCut", sectorDigestCylinderFixture(2, 2)},
	})
}

// TestSectorRouteDigestPointCone covers the point-Loft cone trims and the
// placed copy of a trimmed tooth.
func TestSectorRouteDigestPointCone(t *testing.T) {
	t.Parallel()
	runSectorDigest(t, "point-cone", []sectorDigestFixture{
		{"PointConeToeCut", sectorDigestPointConeFixture(5, true, false)},
		{"PointConeToeInside", sectorDigestPointConeFixture(5, false, false)},
		{"PointConeHeelScrap", sectorDigestPointConeFixture(12, true, false)},
		{"PointConeHeelInside", sectorDigestPointConeFixture(12, false, false)},
		{"PointConeToeThenHeelScrap", sectorDigestPointConeFixture(12, true, true)},
		{"PointConeToeThenHeelInside", sectorDigestPointConeFixture(12, false, true)},
		{"PointConeNormalAndCopyPlacement", func(t *testing.T) []*decad.Body {
			doc := decad.New()
			tooth := pointConeTooth(t, doc)
			cut, err := decad.Cut(t.Context(), tooth, pointConeTool(t, doc, 5))
			require.NoError(t, err)
			move, err := r3.Translation(r3.NewVec(1, 2, 3))
			require.NoError(t, err)
			placed, err := decad.Copy(t.Context(), cut, decad.WithCopyPlacement(move))
			require.NoError(t, err)
			return []*decad.Body{cut, placed}
		}},
	})
}
