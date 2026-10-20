package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/stretchr/testify/require"
)

func TestCurvedHeelCapEndpointsHaveSourceBounds(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	arcs, endpoints := 0, 0
	checkedMutation := false
	for _, pair := range census.Pairs {
		if pair.Verdict != revolvecontact.PairToolArc {
			continue
		}
		arcs++
		vertices, built, buildErr := revolvecontact.CapArcSeamVertices(
			t.Context(), proof, pair.ToolArc)
		require.NoError(t, buildErr)
		require.True(t, built)
		for i, vertex := range vertices {
			endpoints++
			require.Less(t, vertex.BlankBound, 0.1)
			require.Less(t, vertex.LoftBound, 0.1)
			require.Greater(t, vertex.Point.Y, 0.0)
			if i == 0 {
				require.Less(t, vertex.Point.Z, 0.0)
			} else {
				require.Greater(t, vertex.Point.Z, 0.0)
			}
			t.Logf("heel endpoint section=%d side=%d bound=%.9g mm",
				pair.SectionIndex, i, vertex.BlankBound)
		}
		if !checkedMutation {
			changedTime := *pair.ToolArc
			changedTime.Time = lfAdd(changedTime.Time, big.NewRat(1, 1<<20))
			_, accepted, mutationErr := revolvecontact.CapArcSeamVertices(
				t.Context(), proof, &changedTime)
			require.NoError(t, mutationErr)
			require.False(t, accepted)
			changedBox := *pair.ToolArc
			changedBox.NegativeZ.Box[1].Lo = lfAdd(
				changedBox.NegativeZ.Box[1].Lo, big.NewRat(1, 1<<20))
			_, accepted, mutationErr = revolvecontact.CapArcSeamVertices(
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	count := 0
	verticalCount := 0
	slopedCount := 0
	for _, pair := range census.Pairs {
		if pair.Verdict != revolvecontact.PairToolArc {
			continue
		}
		ties, certified, err := revolvecontact.HeelArcSideTiesOf(t.Context(), proof, pair.ToolArc)
		require.NoError(t, err)
		require.True(t, certified, "blank section %d", pair.SectionIndex)
		require.Equal(t, pair.ToolArc.NegativeZ.Ref, ties.NegativeZ.Boundary.Ref)
		require.Equal(t, pair.ToolArc.PositiveZ.Ref, ties.PositiveZ.Boundary.Ref)
		for _, tie := range [...]revolvecontact.CapSideTie{ties.NegativeZ, ties.PositiveZ} {
			require.NotNil(t, tie.Boundary.Face)
			require.True(t, tie.SourceCell != nil || tie.VerticalCell != nil)
			require.False(t, tie.SourceCell != nil && tie.VerticalCell != nil)
			if tie.VerticalCell != nil {
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for _, pair := range census.Pairs {
		if pair.Verdict != revolvecontact.PairToolArc {
			continue
		}
		arc := *pair.ToolArc
		arc.Time = new(big.Rat).Add(arc.Time, big.NewRat(1, 1<<20))
		ties, certified, err := revolvecontact.HeelArcSideTiesOf(t.Context(), proof, &arc)
		require.NoError(t, err)
		require.False(t, certified)
		require.Nil(t, ties)

		arc = *pair.ToolArc
		arc.NegativeZ.Parameter.Lo = new(big.Rat).Add(
			arc.NegativeZ.Parameter.Lo, big.NewRat(1, 1<<60))
		ties, certified, err = revolvecontact.HeelArcSideTiesOf(t.Context(), proof, &arc)
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolvecontact.ContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.Arcs, 2)

	fresh := make([]*revolvecontact.HeelArcSideTies, len(graph.Arcs))
	for i := range graph.Arcs {
		arc := &graph.Arcs[i]
		ties, tied, err := revolvecontact.HeelArcSideTiesOf(t.Context(), proof, arc.Arc)
		require.NoError(t, err)
		require.True(t, tied)
		fresh[i] = ties
		for endpoint, tie := range [...]revolvecontact.CapSideTie{ties.NegativeZ, ties.PositiveZ} {
			id, found := -1, false
			if tie.SourceCell != nil {
				id, found = graph.SourceCellID(arc.SectionIndex, arc.Arc.Circle.BlankFace, *tie.SourceCell)
			} else if tie.VerticalCell != nil {
				id, found = graph.VerticalCellID(arc.SectionIndex, arc.Arc.Circle.BlankFace, *tie.VerticalCell)
			}
			require.True(t, found)
			require.Equal(t, arc.EndpointCellIDs[endpoint], id, "arc %d endpoint %d", i, endpoint)
		}
	}

	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for i := range graph.Arcs {
		arc := graph.Arcs[i].Arc
		ties, tied, err := revolvecontact.HeelArcSideTiesIn(t.Context(), proof, census, arc)
		require.NoError(t, err)
		require.True(t, tied)
		want := [...]revolvecontact.CapSideTie{fresh[i].NegativeZ, fresh[i].PositiveZ}
		for endpoint, got := range [...]revolvecontact.CapSideTie{ties.NegativeZ, ties.PositiveZ} {
			require.True(t, revolvecontact.ToolCapBoundaryEqual(want[endpoint].Boundary, got.Boundary))
			require.True(t, (got.SourceCell != nil) != (got.VerticalCell != nil))
			require.Equal(t, want[endpoint].SourceCell != nil, got.SourceCell != nil)
			if got.SourceCell != nil {
				require.True(t, revolveloftfaces.SameMappedCell(want[endpoint].SourceCell.Mapped, got.SourceCell.Mapped))
				require.Equal(t, want[endpoint].SourceCell.Ref, got.SourceCell.Ref)
			} else {
				require.True(t, revolvecontact.GraphSameVerticalCell(*want[endpoint].VerticalCell, *got.VerticalCell))
			}
		}

		ties, tied, err = revolvecontact.HeelArcSideTiesIn(t.Context(), proof,
			&revolvecontact.PairCensus{}, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		// The verdicts come from the census passed in, not from a fresh one.
		noPairs := *census
		noPairs.Pairs = nil
		ties, tied, err = revolvecontact.HeelArcSideTiesIn(t.Context(), proof, &noPairs, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		moved := *arc
		moved.Time = new(big.Rat).Add(arc.Time, big.NewRat(1, 1<<20))
		ties, tied, err = revolvecontact.HeelArcSideTiesIn(t.Context(), proof, census, &moved)
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

func TestCurvedSourceCapCirclesSplitClosedBlankShell(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolvecontact.AllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	sectionIndex, cellIndex := -1, -1
	for _, side := range manifest.Nonvertical {
		for index, cell := range side.Cells {
			if cell.Mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.Mapped.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact {
				sectionIndex, cellIndex = side.SectionIndex, index
				break
			}
		}
		if cellIndex >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, cellIndex, 0)
	shell, ok, err := revolvecontact.BlankShellWithCapRings(t.Context(), proof,
		sectionIndex, cellIndex)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, shell.CapRings, 2)
	require.Len(t, shell.VertexBounds, len(shell.Mesh.Verts))
	require.Less(t, shell.FaceBound, 0.1)
	require.True(t, shell.EulerTwo())
	edges := make(map[[2]int]struct{})
	for _, triangle := range shell.Mesh.Tris {
		for corner := range 3 {
			a, b := triangle[corner], triangle[(corner+1)%3]
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	require.Equal(t, 2, len(shell.Mesh.Verts)-len(edges)+len(shell.Mesh.Tris))
	vertical, sloped := 0, 0
	for _, ring := range shell.CapRings {
		require.Len(t, ring.Vertices, len(shell.CapRings[0].Vertices))
		require.Equal(t, ring.Section, ring.Arc.Circle.Span)
		require.NotNil(t, ring.Ties)
		require.NotEqual(t, ring.Endpoints[0], ring.Endpoints[1])
		require.GreaterOrEqual(t, len(ring.Occupied), 2)
		require.Equal(t, ring.Endpoints[0], ring.Occupied[0])
		require.Equal(t, ring.Endpoints[1], ring.Occupied[len(ring.Occupied)-1])
		throughAxis := false
		for _, vertex := range ring.Occupied {
			point := shell.Mesh.Verts[vertex]
			throughAxis = throughAxis || point.Y > 0 && point.Z == 0
		}
		require.Equal(t, ring.Arc.ThroughPositiveY, throughAxis)
		for side, endpoint := range ring.Endpoints {
			require.True(t, slices.Contains(ring.Vertices, endpoint))
			require.Less(t, shell.VertexBounds[endpoint], 0.1)
			require.Equal(t, ring.EndpointProof[side].Point, shell.Mesh.Verts[endpoint])
			require.Equal(t, ring.EndpointProof[side].BlankBound,
				shell.VertexBounds[endpoint])
			require.Less(t, ring.EndpointProof[side].LoftBound, 0.1)
		}
		require.True(t, ring.Ties.NegativeZ.SourceCell != nil ||
			ring.Ties.NegativeZ.VerticalCell != nil)
		require.True(t, ring.Ties.PositiveZ.SourceCell != nil ||
			ring.Ties.PositiveZ.VerticalCell != nil)
		x, radius := ring.Arc.Circle.X, ring.Arc.Circle.Radius
		xf, _ := x.Float64()
		rf, _ := radius.Float64()
		span := proof.Section.Spans[ring.Section]
		if span.Start.U == span.End.U {
			vertical++
		} else {
			sloped++
		}
		for _, index := range ring.Vertices {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.Mesh.Verts))
			point := shell.Mesh.Verts[index]
			bound := shell.VertexBounds[index]
			require.InDelta(t, xf, point.X, bound+1e-12)
			require.InDelta(t, rf*rf, point.Y*point.Y+point.Z*point.Z,
				2*rf*bound+bound*bound+1e-12)
			incident := 0
			for _, triangle := range shell.Mesh.Tris {
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
	t.Logf("source-aligned blank shell with %d heel Circles: %d vertices, %d triangles",
		len(shell.CapRings), len(shell.Mesh.Verts), len(shell.Mesh.Tris))
	stale := *shell.CapRings[0].Arc
	stale.Time = new(big.Rat).Add(stale.Time, big.NewRat(1, 1000))
	_, accepted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, []revolvecontact.ShellCut{{
			ShellCut: revolveloftfaces.ShellCut{Section: stale.Circle.Span,
				X: stale.Circle.X, Radius: stale.Circle.Radius},
			Arc: &stale,
		}})
	require.NoError(t, err)
	require.False(t, accepted)
}
