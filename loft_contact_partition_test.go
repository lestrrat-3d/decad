package decad

import (
	"context"
	"maps"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestLoftContactPartitionRealVaryingCapJunctions(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		x1 := 12.0
		if i == 1 {
			x1 = 13
		}
		r := s.CreateRectangle(8, 4, x1, 6)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	lp := loft.n.Payload.(loftPayload)
	prism := internalBoxBody(t, New(), 0, 0, 10, 10, 5)
	pp := prism.n.Payload.(prismPayload)
	for _, operands := range [][2]*Body{{prism, loft}, {loft, prism}} {
		partition, admitted, err := certifyLoftSourcePartition(t.Context(), operands[0], operands[1],
			proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, err)
		require.True(t, admitted)
		require.Equal(t, [2][2]sectionrecord.Point2{
			{{U: 10, V: 4}, {U: 10, V: 6}},
			{{U: 10, V: 4}, {U: 10, V: 6}},
		}, partition.Junction)
		require.Equal(t, [3]string{"0", "4", "5"}, [3]string{
			partition.CapLevels[0].RatString(), partition.CapLevels[1].RatString(),
			partition.CapLevels[2].RatString()})
		require.Positive(t, partition.CapOwners[0][0])
		require.Positive(t, partition.CapOwners[0][1])
		require.Positive(t, partition.CapOwners[0][2]) // overlap belongs to P once
		require.Equal(t, 0, partition.CapOwners[1][0])
		require.Positive(t, partition.CapOwners[1][1])
		require.Equal(t, 0, partition.CapOwners[1][2])
		require.Equal(t, 4, partition.TopWallJoins)
	}
	for capIndex, loftProfile := range []profileRecord{lp.Profile0, lp.Profile1} {
		budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
		scene, tags, _, err := prismcells.BuildSceneRegions(budget,
			[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
			[]prismcells.SceneProfile{{Outer: loftProfile.Outer}},
			&prismcells.Reexpression{Identity: true})
		require.NoError(t, err)
		cells, err := prismcells.ProfilesContext(t.Context(), scene.Profiles)
		require.NoError(t, err)
		classified := cells
		matterA, matterB, resolved, err := prismcells.Classify(budget, tags, classified)
		require.NoError(t, err)
		require.True(t, resolved, "cap %d", capIndex)
		selected := make([]*sketch.Profile, 0, len(classified))
		for i, cell := range classified {
			if capIndex == 0 && (matterA[i] || matterB[i]) ||
				capIndex == 1 && matterB[i] && !matterA[i] {
				selected = append(selected, cell)
			}
		}
		_, _, chain, merged, err := prismcells.MergeWithChain(budget, selected, "Loft cap partition")
		require.NoError(t, err)
		require.True(t, merged, "cap %d selected %d", capIndex, len(selected))
		require.NotEmpty(t, chain)
		junctions := map[[2]float64]bool{}
		for _, cell := range cells {
			require.True(t, cell.Valid, "cap %d", capIndex)
			for i, left := range cell.Outer {
				right := cell.Outer[(i+1)%len(cell.Outer)]
				a, okA := tags[left.Entity]
				b, okB := tags[right.Entity]
				if !okA || !okB || a.IsB == b.IsB {
					continue
				}
				p, ok := scene.CertifyLineLineJunction(left, right)
				if ok {
					junctions[p] = true
				}
			}
		}
		require.True(t, junctions[[2]float64{10, 4}], "cap %d junction count %d", capIndex, len(junctions))
		require.True(t, junctions[[2]float64{10, 6}], "cap %d junction count %d", capIndex, len(junctions))
		require.Len(t, junctions, 2)
	}
}

func loftPartitionRealBodies(t *testing.T, lower, upper [4]float64, z0, z1 float64) (*Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	planes := [2]*sketch.Plane{world.XY(), world.XY()}
	if z0 != 0 {
		var err error
		planes[0], err = world.CreateOffsetPlane(world.XY(), z0)
		require.NoError(t, err)
	}
	var err error
	planes[1], err = world.CreateOffsetPlane(world.XY(), z1)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, coords := range [2][4]float64{lower, upper} {
		s, err := world.CreateSketch(planes[i])
		require.NoError(t, err)
		rect := s.CreateRectangle(coords[0], coords[1], coords[2], coords[3])
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	prism := internalBoxBody(t, New(), 0, 0, 10, 10, 5)
	return prism, loft
}

func TestLoftContactPartitionCapMapRefusesAmbiguousFragments(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	partition, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	budget := proofbound.NewWallWorkBudgetWithOperation(4096, proofbound.NewWorkBudget(t.Context()))
	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	prismLines, ok, err := loftcontact.PartitionLines(budget, pp.Profile.Outer)
	require.NoError(t, err)
	require.True(t, ok)
	loftLines, ok, err := loftcontact.PartitionLines(budget, lp.Profile1.Outer)
	require.NoError(t, err)
	require.True(t, ok)
	scene, tags, _, err := prismcells.BuildSceneRegions(budget,
		[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
		[]prismcells.SceneProfile{{Outer: lp.Profile1.Outer}},
		&prismcells.Reexpression{Identity: true})
	require.NoError(t, err)
	level := big.NewRat(4, 1)
	origins, mapped, err := loftcontact.SceneOrigins(budget, scene, tags, prismLines, loftLines, 1, level)
	require.NoError(t, err)
	require.True(t, mapped)
	require.Len(t, origins, 8)
	var cut, carrierCut [2]*big.Rat
	for k, seam := range partition.Seams {
		cut[k], carrierCut[k], ok = seam.At(big.NewRat(1, 1))
		require.True(t, ok)
	}
	cells, err := prismcells.ProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	complete, err := loftcontact.CertifyCapFragments(budget, scene, cells, origins,
		prismLines, loftLines, partition.Carrier, partition.Crossing, cut, carrierCut, 1, level)
	require.NoError(t, err)
	require.True(t, complete)

	// The top crossing parameter 2/5 is not binary64. Its exact source
	// location is certified from the paired junction and recorded line.
	cutPointFound := false
	var cutCell, cutEdge int
	for ci, cell := range cells {
		for ei, edge := range cell.Outer {
			origin, exists := origins[edge.Entity]
			if !exists || origin.Operand != 1 || !edge.Partial {
				continue
			}
			p, valid := loftcontact.PointOf(sectionrecord.Point2{
				U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
			if !valid {
				continue
			}
			q, valid := loftcontact.PointOf(sectionrecord.Point2{
				U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
			if !valid {
				continue
			}
			checked := &loftcontact.Arithmetic{OK: true}
			a, okA := checked.LineParameter(origin.Line, p)
			b, okB := checked.LineParameter(origin.Line, q)
			if !okA || !okB || a.Cmp(big.NewRat(2, 5)) != 0 && b.Cmp(big.NewRat(2, 5)) != 0 {
				continue
			}
			require.NotEqual(t, 0, proofarith.FloatRat(edge.TStart).Cmp(big.NewRat(2, 5)))
			require.NotEqual(t, 0, proofarith.FloatRat(edge.TEnd).Cmp(big.NewRat(2, 5)))
			cutPointFound, cutCell, cutEdge = true, ci, ei
		}
	}
	require.True(t, cutPointFound)
	altered := append([]*sketch.Profile(nil), cells...)
	copyCell := *cells[cutCell]
	copyCell.Outer = append([]sketch.BoundaryEdge(nil), cells[cutCell].Outer...)
	copyCell.Outer[cutEdge].Partial = false
	altered[cutCell] = &copyCell
	complete, err = loftcontact.CertifyCapFragments(budget, scene, altered, origins,
		prismLines, loftLines, partition.Carrier, partition.Crossing, cut, carrierCut, 1, level)
	require.NoError(t, err)
	require.False(t, complete)
	wrongLevel := make(map[sketch.Entity]loftcontact.CapOrigin, len(origins))
	maps.Copy(wrongLevel, origins)
	for entity, origin := range wrongLevel {
		origin.Level = big.NewRat(5, 1)
		wrongLevel[entity] = origin
		break
	}
	complete, err = loftcontact.CertifyCapFragments(budget, scene, cells, wrongLevel,
		prismLines, loftLines, partition.Carrier, partition.Crossing, cut, carrierCut, 1, level)
	require.NoError(t, err)
	require.False(t, complete)

	duplicateTag := make(map[sketch.Entity]prismcells.Origin, len(tags))
	maps.Copy(duplicateTag, tags)
	for entity, origin := range duplicateTag {
		if origin.IsB {
			origin.Segment = (origin.Segment + 1) % len(loftLines)
			duplicateTag[entity] = origin
			break
		}
	}
	_, mapped, err = loftcontact.SceneOrigins(budget, scene, duplicateTag, prismLines, loftLines, 1, level)
	require.NoError(t, err)
	require.False(t, mapped)
}

func TestLoftContactPartitionInteriorCutIsNotCapChord(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	partition, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	loftLines := loft.n.Payload.(loftPayload).Profile0.Outer.Segments
	found := false
	var seamIndex int
	for k, i := range partition.Crossing {
		line := loftLines[i].(sectionrecord.LineSeg)
		if line.Start.V == 4 && line.End.V == 4 {
			found, seamIndex = true, k
		}
	}
	require.True(t, found)
	seam := partition.Seams[seamIndex]
	start, _, ok := seam.At(big.NewRat(0, 1))
	require.True(t, ok)
	middle, _, ok := seam.At(big.NewRat(1, 2))
	require.True(t, ok)
	end, _, ok := seam.At(big.NewRat(1, 1))
	require.True(t, ok)
	require.Equal(t, "1/2", start.RatString())
	require.Equal(t, "4/9", middle.RatString())
	require.Equal(t, "2/5", end.RatString())
	capChordMiddle := new(big.Rat).Quo(new(big.Rat).Add(start, end), big.NewRat(2, 1))
	require.NotEqual(t, 0, middle.Cmp(capChordMiddle))
}

func TestLoftContactPartitionBentPhysicalSeamKeepsRationalTrace(t *testing.T) {
	seam, admitted, err := loftcontact.CertifyLineSeam(t.Context(), proofbound.NewWorkBudget(t.Context()),
		[2]sectionrecord.Point2{{U: -1, V: 0}, {U: 1, V: 1}},
		[2]sectionrecord.Point2{{U: -1, V: 0}, {U: 3, V: 1}},
		[2]sectionrecord.Point2{{U: 0, V: -1}, {U: 0, V: 2}})
	require.NoError(t, err)
	require.True(t, admitted)
	_, a, ok := seam.At(big.NewRat(0, 1))
	require.True(t, ok)
	_, mid, ok := seam.At(big.NewRat(1, 2))
	require.True(t, ok)
	_, b, ok := seam.At(big.NewRat(1, 1))
	require.True(t, ok)
	require.Equal(t, "1/2", a.RatString())
	require.Equal(t, "4/9", mid.RatString())
	require.Equal(t, "5/12", b.RatString())
	chordMid := new(big.Rat).Quo(new(big.Rat).Add(a, b), big.NewRat(2, 1))
	require.NotEqual(t, 0, mid.Cmp(chordMid))
}

func TestLoftContactPartitionOtherPrismCarrier(t *testing.T) {
	for _, tc := range []struct {
		name         string
		lower, upper [4]float64
		axis         byte
	}{
		{"upper y", [4]float64{4, 8, 6, 12}, [4]float64{4, 8, 6, 13}, 'v'},
		{"left x", [4]float64{-2, 4, 2, 6}, [4]float64{-3, 4, 2, 6}, 'u'},
	} {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := loftPartitionRealBodies(t, tc.lower, tc.upper, 0, 4)
			partition, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
				proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.True(t, admitted)
			carrier := prism.n.Payload.(prismPayload).Profile.Outer.Segments[partition.Carrier].(sectionrecord.LineSeg)
			if tc.axis == 'u' {
				require.Equal(t, 0.0, carrier.Start.U)
				require.Equal(t, 0.0, carrier.End.U)
			} else {
				require.Equal(t, 10.0, carrier.Start.V)
				require.Equal(t, 10.0, carrier.End.V)
			}
		})
	}
}

func TestLoftContactPartitionRealProducerRefusals(t *testing.T) {
	cases := []struct {
		name         string
		lower, upper [4]float64
		z0, z1       float64
	}{
		{"tangent wall", [4]float64{10, 4, 12, 6}, [4]float64{10, 4, 13, 6}, 0, 4},
		{"cap-only tangent", [4]float64{10, 4, 12, 6}, [4]float64{9, 4, 13, 6}, 0, 4},
		{"second carrier", [4]float64{8, 4, 12, 11}, [4]float64{8, 4, 13, 11}, 0, 4},
		{"separate footprints", [4]float64{11, 4, 12, 6}, [4]float64{11, 4, 13, 6}, 0, 4},
		{"shared upper cap", [4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 5},
		{"shifted lower cap", [4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 1, 4},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := loftPartitionRealBodies(t, tc.lower, tc.upper, tc.z0, tc.z1)
			_, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
				proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.False(t, admitted)
		})
	}
}

func TestLoftContactPartitionSourceAndWorkRefusals(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	for _, change := range []struct {
		name string
		edit func(*loftPayload)
	}{
		{"unauthenticated", func(p *loftPayload) { p.AuthenticatedReconstruction = false }},
		{"pairing offset", func(p *loftPayload) { p.Alignment = []int{1} }},
		{"broken source loop", func(p *loftPayload) {
			p.Profile1.Outer.Segments = append([]sectionrecord.CurveSegment(nil), p.Profile1.Outer.Segments...)
			line := p.Profile1.Outer.Segments[0].(sectionrecord.LineSeg)
			line.End.U++
			p.Profile1.Outer.Segments[0] = line
		}},
	} {
		t.Run(change.name, func(t *testing.T) {
			payload := loft.n.Payload.(loftPayload)
			change.edit(&payload)
			alteredBody := bornBody(&Body{n: topo.Body{Payload: payload}})
			_, admitted, err := certifyLoftSourcePartition(t.Context(), prism, alteredBody,
				proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.False(t, admitted)
		})
	}
	exhausted := &proofbound.WorkBudget{StepFn: func() error { return context.DeadlineExceeded }}
	_, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft, exhausted)
	require.ErrorIs(t, err, context.DeadlineExceeded)
	require.False(t, admitted)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, admitted, err = certifyLoftSourcePartition(canceled, prism, loft, proofbound.NewWorkBudget(canceled))
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
}
