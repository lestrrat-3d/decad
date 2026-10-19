package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCurvedHeelCapEndpointsHaveSourceBounds(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	arcs, endpoints := 0, 0
	checkedMutation := false
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		arcs++
		vertices, built, buildErr := revolveLoftFacesCapArcSeamVertices(
			t.Context(), proof, pair.toolArc)
		require.NoError(t, buildErr)
		require.True(t, built)
		for i, vertex := range vertices {
			endpoints++
			require.Less(t, vertex.blankBound, 0.1)
			require.Less(t, vertex.loftBound, 0.1)
			require.Greater(t, vertex.point.Y, 0.0)
			if i == 0 {
				require.Less(t, vertex.point.Z, 0.0)
			} else {
				require.Greater(t, vertex.point.Z, 0.0)
			}
			t.Logf("heel endpoint section=%d side=%d bound=%.9g mm",
				pair.sectionIndex, i, vertex.blankBound)
		}
		if !checkedMutation {
			changedTime := *pair.toolArc
			changedTime.time = lfAdd(changedTime.time, big.NewRat(1, 1<<20))
			_, accepted, mutationErr := revolveLoftFacesCapArcSeamVertices(
				t.Context(), proof, &changedTime)
			require.NoError(t, mutationErr)
			require.False(t, accepted)
			changedBox := *pair.toolArc
			changedBox.negativeZ.box[1].Lo = lfAdd(
				changedBox.negativeZ.box[1].Lo, big.NewRat(1, 1<<20))
			_, accepted, mutationErr = revolveLoftFacesCapArcSeamVertices(
				t.Context(), proof, &changedBox)
			require.NoError(t, mutationErr)
			require.False(t, accepted)
			checkedMutation = true
		}
	}
	require.Equal(t, 2, arcs)
	require.Equal(t, 4, endpoints)
	require.True(t, checkedMutation)
}

func TestCurvedHeelArcEndpointsBindToLiveSideCells(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	count := 0
	verticalCount := 0
	slopedCount := 0
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		ties, certified, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, pair.toolArc)
		require.NoError(t, err)
		require.True(t, certified, "blank section %d", pair.sectionIndex)
		require.Equal(t, pair.toolArc.negativeZ.ref, ties.negativeZ.boundary.ref)
		require.Equal(t, pair.toolArc.positiveZ.ref, ties.positiveZ.boundary.ref)
		for _, tie := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			require.NotNil(t, tie.boundary.face)
			require.True(t, tie.sourceCell != nil || tie.verticalCell != nil)
			require.False(t, tie.sourceCell != nil && tie.verticalCell != nil)
			if tie.verticalCell != nil {
				verticalCount++
			} else {
				slopedCount++
			}
		}
		count++
	}
	require.Equal(t, 2, count)
	require.Equal(t, 2, verticalCount)
	require.Equal(t, 2, slopedCount)
}

func TestCurvedHeelArcSideTiesRejectChangedSourceRecord(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		arc := *pair.toolArc
		arc.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		ties, certified, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, &arc)
		require.NoError(t, err)
		require.False(t, certified)
		require.Nil(t, ties)

		arc = *pair.toolArc
		arc.negativeZ.parameter.Lo = new(big.Rat).Add(
			arc.negativeZ.parameter.Lo, big.NewRat(1, 1<<60))
		ties, certified, err = revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, &arc)
		require.NoError(t, err)
		require.False(t, certified)
		require.Nil(t, ties)
	}
}

func TestCurvedHeelArcSideTieTimeEndpointOwnership(t *testing.T) {
	cell := revolveloftfaces.RevolveLoftFacesMappedCell{TLo: big.NewRat(1, 4), THi: big.NewRat(3, 4)}
	require.True(t, revolveloftfaces.TimeInCell(big.NewRat(1, 4), cell))
	require.True(t, revolveloftfaces.TimeInCell(big.NewRat(1, 2), cell))
	require.True(t, revolveloftfaces.TimeInCell(big.NewRat(3, 4), cell))
	cell.TLoOpen, cell.THiOpen = true, true
	require.False(t, revolveloftfaces.TimeInCell(big.NewRat(1, 4), cell))
	require.True(t, revolveloftfaces.TimeInCell(big.NewRat(1, 2), cell))
	require.False(t, revolveloftfaces.TimeInCell(big.NewRat(3, 4), cell))
}

func TestCurvedHeelArcSideTiesUseCallerCensus(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.arcs, 2)

	fresh := make([]*revolveLoftFacesHeelArcSideTies, len(graph.arcs))
	for i := range graph.arcs {
		arc := &graph.arcs[i]
		ties, tied, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, arc.arc)
		require.NoError(t, err)
		require.True(t, tied)
		fresh[i] = ties
		for endpoint, tie := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			id, found := -1, false
			if tie.sourceCell != nil {
				id, found = graph.sourceCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.sourceCell)
			} else if tie.verticalCell != nil {
				id, found = graph.verticalCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.verticalCell)
			}
			require.True(t, found)
			require.Equal(t, arc.endpointCellIDs[endpoint], id, "arc %d endpoint %d", i, endpoint)
		}
	}

	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for i := range graph.arcs {
		arc := graph.arcs[i].arc
		ties, tied, err := revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, census, arc)
		require.NoError(t, err)
		require.True(t, tied)
		want := [...]revolveLoftFacesCapSideTie{fresh[i].negativeZ, fresh[i].positiveZ}
		for endpoint, got := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			require.True(t, revolveLoftFacesToolCapBoundaryEqual(want[endpoint].boundary, got.boundary))
			require.True(t, (got.sourceCell != nil) != (got.verticalCell != nil))
			require.Equal(t, want[endpoint].sourceCell != nil, got.sourceCell != nil)
			if got.sourceCell != nil {
				require.True(t, revolveLoftFacesSameMappedCell(want[endpoint].sourceCell.mapped, got.sourceCell.mapped))
				require.Equal(t, want[endpoint].sourceCell.ref, got.sourceCell.ref)
			} else {
				require.True(t, revolveLoftFacesGraphSameVerticalCell(*want[endpoint].verticalCell, *got.verticalCell))
			}
		}

		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof,
			&revolveLoftFacesPairCensus{}, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		// The verdicts come from the census passed in, not from a fresh one.
		noPairs := *census
		noPairs.pairs = nil
		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, &noPairs, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		moved := *arc
		moved.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, census, &moved)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)
	}
}

func TestCurvedCapRayOrderRequiresSeparatedSourceBoxes(t *testing.T) {
	point := func(y, z int64, id int) revolveloftfaces.RayOrderRay {
		return revolveloftfaces.RayOrderRay{Y: lfF(float64(y)), Z: lfF(float64(z)),
			SeamIndex: -1, CapCount: 1, CapID: id}
	}
	box := func(y0, y1, z0, z1 int64) revolveloftfaces.RayBox {
		return revolveloftfaces.RayBox{
			Y: loftFaceInterval{Lo: lfF(float64(y0)), Hi: lfF(float64(y1))},
			Z: loftFaceInterval{Lo: lfF(float64(z0)), Hi: lfF(float64(z1))},
		}
	}
	rays := []revolveloftfaces.RayOrderRay{
		{Y: lfOne(), Z: lfZero(), SeamIndex: -1},
		point(2, 1, 0), point(1, 2, 1),
		{Y: lfZero(), Z: lfOne(), SeamIndex: -1},
		{Y: lfSub(lfZero(), lfOne()), Z: lfZero(), SeamIndex: -1},
		{Y: lfZero(), Z: lfSub(lfZero(), lfOne()), SeamIndex: -1},
	}
	boxes := []revolveloftfaces.RayBox{box(2, 2, 1, 1), box(1, 1, 2, 2)}
	require.True(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes))
	boxes[0] = box(2, 2, 1, 5)
	require.False(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes),
		"overlapping source boxes cannot inherit rounded ray order")
	boxes[0] = box(2, 2, 1, 1)
	boxes[1] = box(0, 1, 2, 2)
	require.False(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes),
		"a source box touching the next grid ray must refuse")
}

func TestCurvedCapRayRejectsGridCollapse(t *testing.T) {
	vertex := revolveLoftFacesCapSeamVertex{point: r3.Vec{Y: 1}}
	require.Nil(t, revolveloftfaces.ShellRays(16,
		revolveLoftFacesShellRaySource{caps: []revolveLoftFacesCapSeamVertex{vertex}}))
}

func TestCurvedSourceCapCirclesSplitClosedBlankShell(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	sectionIndex, cellIndex := -1, -1
	for _, side := range manifest.nonvertical {
		for index, cell := range side.cells {
			if cell.mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.mapped.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact {
				sectionIndex, cellIndex = side.sectionIndex, index
				break
			}
		}
		if cellIndex >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, cellIndex, 0)
	shell, ok, err := revolveLoftFacesBlankShellWithCapRings(t.Context(), proof,
		sectionIndex, cellIndex)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, shell.capRings, 2)
	require.Len(t, shell.vertexBounds, len(shell.mesh.Verts))
	require.Less(t, shell.faceBound, 0.1)
	require.True(t, revolveLoftFacesBlankShellEulerTwo(&shell.mesh))
	edges := make(map[[2]int]struct{})
	for _, triangle := range shell.mesh.Tris {
		for corner := range 3 {
			a, b := triangle[corner], triangle[(corner+1)%3]
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	require.Equal(t, 2, len(shell.mesh.Verts)-len(edges)+len(shell.mesh.Tris))
	vertical, sloped := 0, 0
	for _, ring := range shell.capRings {
		require.Len(t, ring.vertices, len(shell.capRings[0].vertices))
		require.Equal(t, ring.sectionIndex, ring.arc.circle.span)
		require.NotNil(t, ring.ties)
		require.NotEqual(t, ring.endpoints[0], ring.endpoints[1])
		require.GreaterOrEqual(t, len(ring.occupied), 2)
		require.Equal(t, ring.endpoints[0], ring.occupied[0])
		require.Equal(t, ring.endpoints[1], ring.occupied[len(ring.occupied)-1])
		throughAxis := false
		for _, vertex := range ring.occupied {
			point := shell.mesh.Verts[vertex]
			throughAxis = throughAxis || point.Y > 0 && point.Z == 0
		}
		require.Equal(t, ring.arc.throughPositiveY, throughAxis)
		for side, endpoint := range ring.endpoints {
			require.True(t, slices.Contains(ring.vertices, endpoint))
			require.Less(t, shell.vertexBounds[endpoint], 0.1)
			require.Equal(t, ring.endpointProof[side].point, shell.mesh.Verts[endpoint])
			require.Equal(t, ring.endpointProof[side].blankBound,
				shell.vertexBounds[endpoint])
			require.Less(t, ring.endpointProof[side].loftBound, 0.1)
		}
		require.True(t, ring.ties.negativeZ.sourceCell != nil ||
			ring.ties.negativeZ.verticalCell != nil)
		require.True(t, ring.ties.positiveZ.sourceCell != nil ||
			ring.ties.positiveZ.verticalCell != nil)
		x, radius := ring.arc.circle.x, ring.arc.circle.radius
		xf, _ := x.Float64()
		rf, _ := radius.Float64()
		span := proof.section.Spans[ring.sectionIndex]
		if span.Start.U == span.End.U {
			vertical++
		} else {
			sloped++
		}
		for _, index := range ring.vertices {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.mesh.Verts))
			point := shell.mesh.Verts[index]
			bound := shell.vertexBounds[index]
			require.InDelta(t, xf, point.X, bound+1e-12)
			require.InDelta(t, rf*rf, point.Y*point.Y+point.Z*point.Z,
				2*rf*bound+bound*bound+1e-12)
			incident := 0
			for _, triangle := range shell.mesh.Tris {
				if slices.Contains(triangle[:], index) {
					incident++
				}
			}
			require.GreaterOrEqual(t, incident, 4,
				"both split meridian bands must reuse cap-ring vertex %d", index)
		}
	}
	require.Equal(t, 1, vertical)
	require.Equal(t, 1, sloped)
	t.Logf("source-aligned blank shell with %d heel circles: %d vertices, %d triangles",
		len(shell.capRings), len(shell.mesh.Verts), len(shell.mesh.Tris))
	stale := *shell.capRings[0].arc
	stale.time = new(big.Rat).Add(stale.time, big.NewRat(1, 1000))
	_, accepted, err := revolveLoftFacesBlankShellAtArcWithCuts(t.Context(), proof, manifest,
		sectionIndex, cellIndex, []revolveLoftFacesShellCut{{
			sectionIndex: stale.circle.span,
			x:            stale.circle.x, radius: stale.circle.radius, arc: &stale,
		}})
	require.NoError(t, err)
	require.False(t, accepted)
}
