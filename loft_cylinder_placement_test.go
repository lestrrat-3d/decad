package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func loftCylinderPlacedTabPair(t *testing.T, lower, upper r3.Frame, opts ...DocumentOption) (*Document, *Body, *Body) {
	t.Helper()
	ctx := t.Context()
	world := sketch.NewWorld()
	bottomPlane, err := world.CreatePlaneFromFrame(lower)
	require.NoError(t, err)
	topPlane, err := world.CreatePlaneFromFrame(upper)
	require.NoError(t, err)
	s0, p0 := loftCylinderTabProfile(t, ctx, world, bottomPlane, 0)
	s1, p1 := loftCylinderTabProfile(t, ctx, world, topPlane, 10*math.Pi/180)
	doc := New(opts...)
	tooth, err := Loft(ctx, doc, s0, p0, s1, p1)
	require.NoError(t, err)
	s, err := world.CreateSketch(bottomPlane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	require.Equal(t, 1, len(s.Profiles()))
	blank, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	return doc, blank, tooth
}

func loftCylinderTestFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return frame
}

func loftCylinderTestTranslation(t *testing.T, z float64) r3.Transform {
	t.Helper()
	move, err := r3.Translation(r3.Vec{Z: z})
	require.NoError(t, err)
	return move
}

func loftCylinderErrorText(err error) string {
	if err == nil {
		return "<nil>"
	}
	message := err.Error()
	if len(message) > 160 {
		return message[:160]
	}
	return message
}

func loftCylinderCheckPlacedResult(t *testing.T, joined, blank, tooth *Body, placed bool) {
	t.Helper()
	require.True(t, joined.IsSolid())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Positive(t, volume.Value.Base())
	box, err := joined.Bounds()
	require.NoError(t, err)
	require.True(t, proofFiniteBox(box), "result bounds must be finite")
	require.Less(t, box.Min.X, box.Max.X)
	require.Less(t, box.Min.Y, box.Max.Y)
	require.Less(t, box.Min.Z, box.Max.Z)
	fp, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.LoftCylinderJoin)
	require.LessOrEqual(t, fp.MeshBound, 0.1)
	require.True(t, fp.ContactAudited)
	require.Equal(t, blank.originProducer(), bodyHandle(fp.LoftCylinderJoin.Blank).originProducer())
	require.Equal(t, 1, len(fp.LoftCylinderJoin.Teeth))
	require.Equal(t, tooth.originProducer(), bodyHandle(fp.LoftCylinderJoin.Teeth[0]).originProducer())
	require.Equal(t, placed, fp.LoftCylinderJoin.Placement != nil)
	seenBlank, seenTooth := false, false
	for _, face := range joined.Faces() {
		for _, origin := range face.Origins() {
			seenBlank = seenBlank || origin.producer == blank.originProducer()
			seenTooth = seenTooth || origin.producer == tooth.originProducer()
		}
	}
	require.True(t, seenBlank, "result lost blank face provenance")
	require.True(t, seenTooth, "result lost tooth face provenance")
}

func proofFiniteBox(box Box) bool {
	for _, value := range []float64{box.Min.X, box.Min.Y, box.Min.Z,
		box.Max.X, box.Max.Y, box.Max.Z, box.Bound.Base()} {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func loftCylinderReadingsOverlap(t *testing.T, name string, actual, local Measurement) {
	t.Helper()
	a, b := actual.Value.Base(), local.Value.Base()
	ab, bb := actual.Bound.Base(), local.Bound.Base()
	require.False(t, math.IsNaN(a) || math.IsInf(a, 0) || math.IsNaN(b) || math.IsInf(b, 0),
		"%s has a nonfinite reading", name)
	require.False(t, math.IsNaN(ab) || math.IsInf(ab, 0) || math.IsNaN(bb) || math.IsInf(bb, 0),
		"%s has a nonfinite bound", name)
	require.GreaterOrEqual(t, ab, 0.0, "%s actual bound is negative", name)
	require.GreaterOrEqual(t, bb, 0.0, "%s canonical bound is negative", name)
	gap := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(a), proofarith.FloatRat(b)))
	allow := new(big.Rat).Add(proofarith.FloatRat(ab), proofarith.FloatRat(bb))
	require.True(t, gap.Cmp(allow) <= 0, "%s exceeds published bounds: gap=%s allowance=%s",
		name, gap.FloatString(12), allow.FloatString(12))
}

func loftCylinderPlacedBoxOverlap(t *testing.T, actual, local Box, pose r3.Transform) {
	t.Helper()
	require.True(t, proofFiniteBox(actual), "placed box has a nonfinite reading")
	require.True(t, proofFiniteBox(local), "canonical box has a nonfinite reading")
	require.GreaterOrEqual(t, actual.Bound.Base(), 0.0, "placed box bound is negative")
	require.GreaterOrEqual(t, local.Bound.Base(), 0.0, "canonical box bound is negative")
	mapOf := loftcylinder.AffineOf(pose)
	var lo, hi [3]*big.Rat
	for corner := range 8 {
		coordinates := [3]float64{local.Min.X, local.Min.Y, local.Min.Z}
		upper := [3]float64{local.Max.X, local.Max.Y, local.Max.Z}
		for j := range 3 {
			if corner&(1<<j) != 0 {
				coordinates[j] = upper[j]
			}
		}
		for i := range 3 {
			value := new(big.Rat).Set(mapOf.Shift[i])
			for j := range 3 {
				value.Add(value, new(big.Rat).Mul(mapOf.Linear[i][j], proofarith.FloatRat(coordinates[j])))
			}
			if lo[i] == nil || value.Cmp(lo[i]) < 0 {
				lo[i] = value
			}
			if hi[i] == nil || value.Cmp(hi[i]) > 0 {
				hi[i] = value
			}
		}
	}
	allow := new(big.Rat).Add(proofarith.FloatRat(actual.Bound.Base()),
		proofarith.FloatRat(local.Bound.Base()))
	actualLo := [3]float64{actual.Min.X, actual.Min.Y, actual.Min.Z}
	actualHi := [3]float64{actual.Max.X, actual.Max.Y, actual.Max.Z}
	for i, axis := range []string{"X", "Y", "Z"} {
		minGap := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(actualLo[i]), lo[i]))
		maxGap := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(actualHi[i]), hi[i]))
		require.True(t, minGap.Cmp(allow) <= 0, "%s minimum exceeds published box bounds", axis)
		require.True(t, maxGap.Cmp(allow) <= 0, "%s maximum exceeds published box bounds", axis)
	}
}

func loftCylinderCheckCanonicalReadings(t *testing.T, world *Body, local *Body, pose r3.Transform) {
	t.Helper()
	worldVolume, err := world.Volume()
	require.NoError(t, err)
	localVolume, err := local.Volume()
	require.NoError(t, err)
	loftCylinderReadingsOverlap(t, "volume", worldVolume, localVolume)
	worldArea, err := world.Area()
	require.NoError(t, err)
	localArea, err := local.Area()
	require.NoError(t, err)
	loftCylinderReadingsOverlap(t, "area", worldArea, localArea)
	worldBox, err := world.Bounds()
	require.NoError(t, err)
	localBox, err := local.Bounds()
	require.NoError(t, err)
	loftCylinderPlacedBoxOverlap(t, worldBox, localBox, pose)
}

// The seven source arrangements match the retained gallery diagnostic's
// route cases. All placed cases must use the structural certificate.
func TestLoftCylinderUnionCommonPlacement(t *testing.T) {
	xyU, xyV := r3.Vec{X: 1}, r3.Vec{Y: 1}
	for _, tc := range []struct {
		name         string
		lower, upper r3.Frame
		translate    float64
		placed       bool
	}{
		{"origin", loftCylinderTestFrame(t, r3.Vec{}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 2}, xyU, xyV), 0, false},
		{"translation_1e-12", loftCylinderTestFrame(t, r3.Vec{}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 2}, xyU, xyV), 1e-12, true},
		{"translation_1e-3", loftCylinderTestFrame(t, r3.Vec{}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 2}, xyU, xyV), 1e-3, true},
		{"translation_7", loftCylinderTestFrame(t, r3.Vec{}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 2}, xyU, xyV), 7, true},
		{"native_identity", loftCylinderTestFrame(t, r3.Vec{}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 2}, xyU, xyV), 0, false},
		{"native_shift_7", loftCylinderTestFrame(t, r3.Vec{Z: 7}, xyU, xyV),
			loftCylinderTestFrame(t, r3.Vec{Z: 9}, xyU, xyV), 0, true},
		{"native_rotated_yz", loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1}),
			loftCylinderTestFrame(t, r3.Vec{X: 2}, r3.Vec{Y: 1}, r3.Vec{Z: 1}), 0, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, blank, tooth := loftCylinderPlacedTabPair(t, tc.lower, tc.upper)
			if tc.translate != 0 {
				motion := loftCylinderTestTranslation(t, tc.translate)
				var err error
				blank, err = Transform(t.Context(), blank, motion)
				require.NoError(t, err)
				tooth, err = Transform(t.Context(), tooth, motion)
				require.NoError(t, err)
			}
			joined, err := Union(t.Context(), blank, tooth)
			require.NoError(t, err)
			loftCylinderCheckPlacedResult(t, joined, blank, tooth, tc.placed)
			if tc.placed {
				fp := joined.n.Payload.(facetedPayload)
				placement := fp.LoftCylinderJoin.Placement
				localJoin, admitted, err := tryLoftCylinderUnionTeeth(t.Context(), doc,
					joined.originProducer(), bodyHandle(placement.Blank), bodyHandles(placement.Teeth))
				require.NoError(t, err)
				require.True(t, admitted, "canonical source bodies must rebuild the join")
				loftCylinderCheckCanonicalReadings(t, joined,
					localJoin, placement.Pose)
				if tc.name == "translation_1e-12" {
					moved, err := Transform(t.Context(), joined, loftCylinderTestTranslation(t, 1))
					require.NoError(t, err)
					movedPayload := moved.n.Payload.(facetedPayload)
					require.Nil(t, movedPayload.LoftCylinderJoin,
						"public Transform must clear the structural record")
				}
			}
		})
	}
}

func TestLoftCylinderCommonPlacementConsumers(t *testing.T) {
	ctx := t.Context()
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	doc, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(180))
	require.NoError(t, err)
	second, err := Copy(ctx, tooth, WithCopyPlacement(turn))
	require.NoError(t, err)
	move := loftCylinderTestTranslation(t, 7)
	blank, err = Transform(ctx, blank, move)
	require.NoError(t, err)
	tooth, err = Transform(ctx, tooth, move)
	require.NoError(t, err)
	second, err = Transform(ctx, second, move)
	require.NoError(t, err)
	firstJoin, err := Union(ctx, blank, tooth)
	require.NoError(t, err)
	loftCylinderCheckPlacedResult(t, firstJoin, blank, tooth, true)
	joined, err := Union(ctx, firstJoin, second)
	require.NoError(t, err)
	fp, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.LoftCylinderJoin)
	require.NotNil(t, fp.LoftCylinderJoin.Placement)
	require.Equal(t, 2, len(fp.LoftCylinderJoin.Teeth))
	require.Equal(t, second.originProducer(), bodyHandle(fp.LoftCylinderJoin.Teeth[1]).originProducer())
	require.LessOrEqual(t, fp.MeshBound, 0.1)
	placement := fp.LoftCylinderJoin.Placement
	localJoin, admitted, err := tryLoftCylinderUnionTeeth(ctx, doc,
		joined.originProducer(), bodyHandle(placement.Blank), bodyHandles(placement.Teeth))
	require.NoError(t, err)
	require.True(t, admitted, "canonical source bodies must rebuild the later join")
	loftCylinderCheckCanonicalReadings(t, joined, localJoin, placement.Pose)
	seenJoin := map[producerID]bool{}
	for _, face := range joined.Faces() {
		for _, origin := range face.Origins() {
			seenJoin[origin.producer] = true
		}
	}
	for _, source := range []*Body{blank, tooth, second} {
		require.True(t, seenJoin[source.originProducer()], "later join lost source %d", source.originProducer())
	}
	copyOfJoin, err := Copy(ctx, joined)
	require.NoError(t, err)
	copyPayload := copyOfJoin.n.Payload.(facetedPayload)
	require.Nil(t, copyPayload.LoftCylinderJoin, "public Copy must clear the structural record")

	w := sketch.NewWorld()
	base, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(base)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 0.5)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	require.Equal(t, 1, len(s.Profiles()))
	tool, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	localBore, admitted, err := tryLoftCylinderBoreCut(ctx, meshbool.OpCut, doc,
		tool.originProducer(), localJoin, tool)
	require.NoError(t, err)
	require.True(t, admitted)
	tool, err = Transform(ctx, tool, move)
	require.NoError(t, err)
	bored, err := Cut(ctx, joined, tool)
	require.NoError(t, err)
	require.True(t, bored.IsSolid())
	cut, ok := bored.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.LessOrEqual(t, cut.MeshBound, 0.1)
	require.True(t, cut.ContactAudited)
	volume, err := bored.Volume()
	require.NoError(t, err)
	require.Positive(t, volume.Value.Base())
	box, err := bored.Bounds()
	require.NoError(t, err)
	require.True(t, proofFiniteBox(box), "bore result bounds must be finite")
	loftCylinderCheckCanonicalReadings(t, bored, localBore, placement.Pose)
	seenBore := map[producerID]bool{}
	for _, face := range bored.Faces() {
		for _, origin := range face.Origins() {
			seenBore[origin.producer] = true
		}
	}
	for _, source := range []*Body{blank, tooth, second, tool} {
		require.True(t, seenBore[source.originProducer()], "bore lost source %d", source.originProducer())
	}
}

func TestLoftCylinderCommonPlacementDifferentLoftSourceFallsBack(t *testing.T) {
	ctx := t.Context()
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	doc, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)

	// A second real Loft has matching shape but a distinct authenticated source.
	world := sketch.NewWorld()
	bottomPlane, err := world.CreatePlaneFromFrame(lower)
	require.NoError(t, err)
	topPlane, err := world.CreatePlaneFromFrame(upper)
	require.NoError(t, err)
	s0, p0 := loftCylinderTabProfile(t, ctx, world, bottomPlane, 0)
	s1, p1 := loftCylinderTabProfile(t, ctx, world, topPlane, 10*math.Pi/180)
	other, err := Loft(ctx, doc, s0, p0, s1, p1)
	require.NoError(t, err)
	firstSource := tooth.n.Payload.(loftPayload).SourceIdentity
	otherSource := other.n.Payload.(loftPayload).SourceIdentity
	require.True(t, firstSource != otherSource, "independent Lofts must have distinct source identities")

	move := loftCylinderTestTranslation(t, 7)
	blank, err = Transform(ctx, blank, move)
	require.NoError(t, err)
	tooth, err = Transform(ctx, tooth, move)
	require.NoError(t, err)
	other, err = Transform(ctx, other, move)
	require.NoError(t, err)
	joined, err := Union(ctx, blank, tooth)
	require.NoError(t, err)
	loftCylinderCheckPlacedResult(t, joined, blank, tooth, true)

	joinedPayload := joined.n.Payload.(facetedPayload)
	require.False(t, loftcylinder.SourceIdentitiesMatch(bodyNodes([]*Body{tooth, other})),
		"different Loft sources must miss continuation before private reconstruction")
	admitted, err := tryPlacedLoftCylinderUnion(ctx, doc, other.originProducer(), joined, other)
	require.NoError(t, err)
	require.False(t, admitted, "different Loft sources must use the generic Union")
	require.NotNil(t, joinedPayload.LoftCylinderJoin.Placement,
		"fallback must leave the existing placed certificate intact")

	result, err := Union(ctx, joined, other)
	if err != nil {
		require.True(t, errors.Is(err, ErrUnsupported) || errors.Is(err, ErrBooleanFailed),
			"generic route returned an unexpected error: %s", loftCylinderErrorText(err))
		return
	}
	if fp, ok := result.n.Payload.(facetedPayload); ok {
		require.Nil(t, fp.LoftCylinderJoin, "different Loft sources used the placed certificate")
	}
}

func TestLoftCylinderCommonPlacementBoreEligibilityPrecedesRebuild(t *testing.T) {
	ctx := t.Context()
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	doc, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	move := loftCylinderTestTranslation(t, 7)
	var err error
	blank, err = Transform(ctx, blank, move)
	require.NoError(t, err)
	tooth, err = Transform(ctx, tooth, move)
	require.NoError(t, err)
	joined, err := Union(ctx, blank, tooth)
	require.NoError(t, err)
	loftCylinderCheckPlacedResult(t, joined, blank, tooth, true)

	makeTool := func(radius, start, height, centerX float64) *Body {
		t.Helper()
		world := sketch.NewWorld()
		plane, err := world.CreateOffsetPlane(world.XY(), start)
		require.NoError(t, err)
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(centerX, 0)
		s.Fix(center)
		s.CreateCircle(center, radius)
		_, err = s.Solve(ctx)
		require.NoError(t, err)
		require.Equal(t, 1, len(s.Profiles()))
		tool, err := Extrude(ctx, doc, s, s.Profiles()[0],
			Distance{D: units.Millimeters(height), Dir: Along})
		require.NoError(t, err)
		tool, err = Transform(ctx, tool, move)
		require.NoError(t, err)
		return tool
	}
	wide := makeTool(3.5, -1, 4, 0)
	short := makeTool(0.5, 0, 1, 0)
	offCenter := makeTool(0.5, -1, 4, 0.2)
	valid := makeTool(0.5, -1, 4, 0)
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	for _, tc := range []struct {
		name string
		tool *Body
	}{{"radius", wide}, {"axial span", short}, {"center", offCenter}} {
		t.Run(tc.name, func(t *testing.T) {
			admitted, err := tryPlacedLoftCylinderBoreCut(cancelled, doc,
				tc.tool.originProducer(), joined, tc.tool)
			require.NoError(t, err)
			require.False(t, admitted,
				"ineligible %s tool must return to generic Cut before rebuilding", tc.name)
		})
	}
	admitted, err := tryPlacedLoftCylinderBoreCut(cancelled, doc,
		valid.originProducer(), joined, valid)
	require.False(t, admitted)
	require.ErrorIs(t, err, context.Canceled,
		"an eligible tool must preserve the reconstruction's cancellation error")
}

func TestLoftCylinderCommonPlacementNativeYZConsumers(t *testing.T) {
	ctx := t.Context()
	y, z := r3.Vec{Y: 1}, r3.Vec{Z: 1}
	lower := loftCylinderTestFrame(t, r3.Vec{}, y, z)
	upper := loftCylinderTestFrame(t, r3.Vec{X: 2}, y, z)
	doc, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	turnFrame := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: -1})
	turn, err := r3.FromFrame(turnFrame)
	require.NoError(t, err)
	second, err := Copy(ctx, tooth, WithCopyPlacement(turn))
	require.NoError(t, err)
	firstJoin, err := Union(ctx, blank, tooth)
	require.NoError(t, err)
	loftCylinderCheckPlacedResult(t, firstJoin, blank, tooth, true)
	joined, err := Union(ctx, firstJoin, second)
	require.NoError(t, err)
	fp, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.LoftCylinderJoin)
	require.NotNil(t, fp.LoftCylinderJoin.Placement)
	require.Equal(t, 2, len(fp.LoftCylinderJoin.Teeth))
	placement := fp.LoftCylinderJoin.Placement
	localJoin, admitted, err := tryLoftCylinderUnionTeeth(ctx, doc,
		joined.originProducer(), bodyHandle(placement.Blank), bodyHandles(placement.Teeth))
	require.NoError(t, err)
	require.True(t, admitted, "canonical source bodies must rebuild the native YZ join")
	loftCylinderCheckCanonicalReadings(t, joined, localJoin, placement.Pose)

	w := sketch.NewWorld()
	toolFrame := loftCylinderTestFrame(t, r3.Vec{X: -1}, y, z)
	toolPlane, err := w.CreatePlaneFromFrame(toolFrame)
	require.NoError(t, err)
	s, err := w.CreateSketch(toolPlane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 0.5)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	require.Equal(t, 1, len(s.Profiles()))
	tool, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	bored, err := Cut(ctx, joined, tool)
	require.NoError(t, err)
	require.True(t, bored.IsSolid())
	box, err := bored.Bounds()
	require.NoError(t, err)
	require.True(t, proofFiniteBox(box), "native YZ bore bounds must be finite")
	seen := map[producerID]bool{}
	for _, face := range bored.Faces() {
		for _, origin := range face.Origins() {
			seen[origin.producer] = true
		}
	}
	for _, source := range []*Body{blank, tooth, second, tool} {
		require.True(t, seen[source.originProducer()], "native YZ bore lost source %d", source.originProducer())
	}
}

func TestLoftCylinderCommonPlacementRefusesUnequalPoses(t *testing.T) {
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	_, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper, ExactOnly())
	var err error
	blank, err = Transform(t.Context(), blank, loftCylinderTestTranslation(t, 7))
	require.NoError(t, err)
	tooth, err = Transform(t.Context(), tooth, loftCylinderTestTranslation(t, 7.001))
	require.NoError(t, err)
	admitted, err := tryPlacedLoftCylinderUnion(t.Context(), documentHandle(blank.n.Doc),
		blank.originProducer(), blank, tooth)
	require.NoError(t, err)
	require.False(t, admitted, "unequal source poses must miss the structural certificate")
	_, err = Union(t.Context(), blank, tooth)
	require.True(t, errors.Is(err, ErrUnsupported),
		"generic route should still refuse this near-contact pair: %s", loftCylinderErrorText(err))
}

func TestLoftCylinderCommonPlacementLeavesArbitraryAngleGeneric(t *testing.T) {
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	_, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	pose, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(15))
	require.NoError(t, err)
	blank, err = Transform(t.Context(), blank, pose)
	require.NoError(t, err)
	tooth, err = Transform(t.Context(), tooth, pose)
	require.NoError(t, err)
	admitted, err := tryPlacedLoftCylinderUnion(t.Context(), documentHandle(blank.n.Doc),
		blank.originProducer(), blank, tooth)
	require.NoError(t, err)
	require.False(t, admitted, "an arbitrary-angle pose needs its own source-map proof")
	joined, err := Union(t.Context(), blank, tooth)
	if err != nil {
		require.True(t, errors.Is(err, ErrUnsupported),
			"generic route returned an unexpected error: %s", loftCylinderErrorText(err))
		return
	}
	fp, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.Nil(t, fp.LoftCylinderJoin, "arbitrary-angle result used the structural certificate")
}

func TestLoftCylinderCommonPlacementRejectsOrdinaryQuarterTurn(t *testing.T) {
	quarter, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	require.False(t, loftcylinder.SignedPose(quarter),
		"the current r3.Rotation quarter-turn has rounded sine/cosine entries")
}

func TestLoftCylinderCommonPlacementRejectsRoundedComposition(t *testing.T) {
	first := loftCylinderTestTranslation(t, 0.1)
	second := loftCylinderTestTranslation(t, 0.2)
	pose, err := first.Then(second)
	require.NoError(t, err)
	require.True(t, loftcylinder.SignedPose(pose))
	require.False(t, loftcylinder.AffineOf(first).Then(loftcylinder.AffineOf(second)).
		Equals(loftcylinder.AffineOf(pose)),
		"a rounded native height/translation composition needs a source-map allowance")
	lower := loftCylinderTestFrame(t, r3.Vec{Z: 0.1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2.1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	doc, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	blank, err = Transform(t.Context(), blank, second)
	require.NoError(t, err)
	tooth, err = Transform(t.Context(), tooth, second)
	require.NoError(t, err)
	admitted, err := tryPlacedLoftCylinderUnion(t.Context(), doc,
		blank.originProducer(), blank, tooth)
	require.NoError(t, err)
	require.False(t, admitted, "rounded source-map composition must miss the structural path")
	joined, err := Union(t.Context(), blank, tooth)
	if err != nil {
		require.True(t, errors.Is(err, ErrUnsupported),
			"generic route returned an unexpected error: %s", loftCylinderErrorText(err))
		return
	}
	if fp, ok := joined.n.Payload.(facetedPayload); ok {
		require.Nil(t, fp.LoftCylinderJoin, "rounded composition used the structural certificate")
	}
}

func TestLoftCylinderCommonPlacementLeavesOtherPrismsGeneric(t *testing.T) {
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	doc, blank, _ := loftCylinderPlacedTabPair(t, lower, upper)
	other, err := Copy(t.Context(), blank)
	require.NoError(t, err)
	admitted, err := tryPlacedLoftCylinderUnion(t.Context(), doc,
		blank.originProducer(), blank, other)
	require.NoError(t, err)
	require.False(t, admitted, "a prism pair must miss the cylinder/Loft path")
	joined, err := Union(t.Context(), blank, other)
	if err != nil {
		require.True(t, errors.Is(err, ErrUnsupported) || errors.Is(err, ErrBooleanFailed),
			"other prism route returned an unexpected error: %s", loftCylinderErrorText(err))
		return
	}
	if fp, ok := joined.n.Payload.(facetedPayload); ok {
		require.Nil(t, fp.LoftCylinderJoin, "a prism pair used the cylinder/Loft certificate")
	}
}

func TestLoftCylinderCommonPlacementLargeRepeatedShift(t *testing.T) {
	lower := loftCylinderTestFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	upper := loftCylinderTestFrame(t, r3.Vec{Z: 2}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	_, blank, tooth := loftCylinderPlacedTabPair(t, lower, upper)
	for _, z := range []float64{1e6, 7} {
		move := loftCylinderTestTranslation(t, z)
		var err error
		blank, err = Transform(t.Context(), blank, move)
		require.NoError(t, err)
		tooth, err = Transform(t.Context(), tooth, move)
		require.NoError(t, err)
	}
	joined, err := Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	loftCylinderCheckPlacedResult(t, joined, blank, tooth, true)
	box, err := joined.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(box.Min.Z-1_000_007), box.Bound.Base())
	require.LessOrEqual(t, math.Abs(box.Max.Z-1_000_009), box.Bound.Base())
}
