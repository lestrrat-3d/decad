package decad

import (
	"context"
	"fmt"
	"math/big"
	"reflect"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCurvedContactGraphHeldAnchors(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolvecontact.ContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.Pairs, len(blank.Faces())*len(trimmed.Faces()))
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.Pairs, len(census.Pairs))
	for i, pair := range census.Pairs {
		require.Equal(t, pair.Verdict, graph.Pairs[i].Verdict)
		require.Equal(t, pair.BlankFace, graph.Pairs[i].BlankFace)
		require.Equal(t, pair.TrimmedFace, graph.Pairs[i].TrimmedFace)
		require.Len(t, graph.Pairs[i].CellIDs,
			len(pair.SourceCells)+len(pair.VerticalCells))
	}
	expectedCells := 0
	for _, side := range census.Manifest.Nonvertical {
		expectedCells += len(side.Cells)
	}
	for _, side := range census.Manifest.Vertical {
		if side.Side != nil {
			expectedCells += len(side.Side.Cells)
		}
	}
	require.Len(t, graph.Cells, expectedCells)
	seenCells := make(map[int]bool)
	for pairID, pair := range graph.Pairs {
		for _, cellID := range pair.CellIDs {
			require.False(t, seenCells[cellID])
			seenCells[cellID] = true
			require.Equal(t, pairID, graph.Cells[cellID].PairID)
			require.Equal(t, pair.BlankFace, graph.Cells[cellID].BlankFace)
			require.Equal(t, pair.TrimmedFace, graph.Cells[cellID].TrimmedFace)
		}
	}
	for cellID, cell := range graph.Cells {
		if cell.TrimmedFace == nil {
			require.Equal(t, -1, cell.PairID)
			require.False(t, seenCells[cellID])
		} else {
			require.True(t, seenCells[cellID])
		}
	}
	require.Len(t, graph.Runs, 2)
	seenEventIDs := make(map[int]bool)
	for _, arc := range graph.Arcs {
		for _, id := range arc.EndpointIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
	}
	seenStationIDs := make(map[int]bool)
	for _, run := range graph.Runs {
		for _, id := range run.StationIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
		cell := graph.Cells[run.CellID]
		require.NotNil(t, cell.Source)
		mapped := cell.Source.Mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolvecontact.CertifiedArcPatches(t.Context(),
			bodyNode(blank), bodyNode(trimmed), cell.SectionIndex, mapped.Segment, mapped.Band,
			loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, run.Stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		shell, built, shellErr := revolvecontact.BlankShellWithCapRings(t.Context(), proof,
			cell.SectionIndex, cell.CellIndex)
		require.NoError(t, shellErr)
		require.True(t, built)
		require.Equal(t, run.StationIDs, shell.SeamIDs)
		require.Len(t, run.StationIDs, len(run.Stations))
		seenShellVertices := make(map[int]bool)
		for i, id := range run.StationIDs {
			require.False(t, seenStationIDs[id])
			seenStationIDs[id] = true
			shellIndex, blankIndex, loftIndex := shell.Seam[i], blankPatch.Seam[i], loftPatch.Seam[i]
			require.False(t, seenShellVertices[shellIndex])
			seenShellVertices[shellIndex] = true
			require.Equal(t, seam[i].Point, shell.Mesh.Verts[shellIndex])
			require.Equal(t, seam[i].Point, blankPatch.Vertices[blankIndex])
			require.Equal(t, seam[i].Point, loftPatch.Vertices[loftIndex])
			require.GreaterOrEqual(t, shell.VertexBounds[shellIndex], seam[i].BlankBound)
			require.GreaterOrEqual(t, blankPatch.VertexBounds[blankIndex], seam[i].BlankBound)
			require.GreaterOrEqual(t, loftPatch.VertexBounds[loftIndex], seam[i].LoftBound)
		}
		require.Len(t, shell.CapRings, len(graph.Arcs))
		seenEndpointIDs, seenRings := make(map[int]bool), make(map[int]bool)
		for _, arc := range graph.Arcs {
			matched := -1
			for i, ring := range shell.CapRings {
				if revolvecontact.ToolCapOccupiedArcEqual(arc.Arc, ring.Arc) {
					require.Equal(t, -1, matched)
					matched = i
				}
			}
			require.GreaterOrEqual(t, matched, 0)
			require.False(t, seenRings[matched])
			seenRings[matched] = true
			ring := shell.CapRings[matched]
			require.Equal(t, arc.EndpointIDs, ring.EndpointGraphIDs)
			require.Equal(t, arc.EndpointCellIDs, ring.EndpointCellIDs)
			for endpoint, cellID := range arc.EndpointCellIDs {
				id := arc.EndpointIDs[endpoint]
				require.False(t, seenEndpointIDs[id])
				seenEndpointIDs[id] = true
				require.Equal(t, ring.EndpointProof[endpoint].Point,
					shell.Mesh.Verts[ring.Endpoints[endpoint]])
				require.GreaterOrEqual(t, graph.Cells[cellID].PairID, 0)
				require.True(t, graph.EndpointCellMatches(arc, &ring, endpoint, cellID))
			}
		}
		require.Len(t, seenRings, len(graph.Arcs))
	}
	require.Len(t, seenStationIDs, 36)
	require.Len(t, seenEventIDs, 36+2*len(graph.Arcs))
	current, err := revolvecontact.ContactGraphCurrent(t.Context(), graph)
	require.NoError(t, err)
	require.True(t, current)
}

func TestCurvedContactGraphRejectsStaleSourceEvents(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolvecontact.ContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	current := func() bool {
		live, liveErr := revolvecontact.ContactGraphCurrent(t.Context(), graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.Cells
		graph.Cells = slices.Clone(original[:len(original)-1])
		require.False(t, current())
		graph.Cells = original
	})
	t.Run("duplicate endpoint owner", func(t *testing.T) {
		original := graph.Arcs[0].EndpointCellIDs
		graph.Arcs[0].EndpointCellIDs[1] = original[0]
		require.False(t, current())
		graph.Arcs[0].EndpointCellIDs = original
	})
	t.Run("cross-kind event ID", func(t *testing.T) {
		original := graph.Arcs[0].EndpointIDs
		graph.Arcs[0].EndpointIDs[0] = graph.Runs[0].StationIDs[0]
		require.False(t, current())
		graph.Arcs[0].EndpointIDs = original
	})
	t.Run("swapped face", func(t *testing.T) {
		original := graph.Cells[0].TrimmedFace
		graph.Cells[0].TrimmedFace = graph.Pairs[0].TrimmedFace
		if graph.Cells[0].TrimmedFace == original {
			graph.Cells[0].TrimmedFace = graph.Pairs[1].TrimmedFace
		}
		require.False(t, current())
		graph.Cells[0].TrimmedFace = original
	})
	t.Run("changed source ref", func(t *testing.T) {
		original := graph.Cells[0].Source.Ref
		graph.Cells[0].Source.Ref = provenance.Ref{Role: "changed graph source"}
		require.False(t, current())
		graph.Cells[0].Source.Ref = original
	})
	t.Run("changed open interval", func(t *testing.T) {
		original := graph.Cells[0].Source.Mapped.TLoOpen
		graph.Cells[0].Source.Mapped.TLoOpen = !original
		require.False(t, current())
		graph.Cells[0].Source.Mapped.TLoOpen = original
	})
	t.Run("changed live station", func(t *testing.T) {
		original := trim.Source.SourceStation[0]
		trim.Source.SourceStation[0].U += 0.001
		require.False(t, current())
		trim.Source.SourceStation[0] = original
	})
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.Arcs[0].Arc.NegativeZ.Parameter.Lo
		graph.Arcs[0].Arc.NegativeZ.Parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.Arcs[0].Arc.NegativeZ.Parameter.Lo = original
	})
	require.True(t, current())
}

func TestCurvedSourcePairCensusCoversLiveFaces(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, census.Manifest)
	require.Len(t, census.Pairs, len(blank.Faces())*len(trimmed.Faces()))

	seen := make(map[[2]*Face]struct{})
	counts := make(map[revolvecontact.PairVerdict]int)
	for _, pair := range census.Pairs {
		key := [2]*Face{faceHandle(pair.BlankFace), faceHandle(pair.TrimmedFace)}
		_, duplicate := seen[key]
		require.False(t, duplicate, "a live face pair must occur once")
		seen[key] = struct{}{}
		require.Equal(t, proof.Section.Spans[pair.SectionIndex].Face, pair.BlankFace)
		counts[pair.Verdict]++
		switch pair.Verdict {
		case revolvecontact.PairMiss:
			require.Nil(t, pair.ToolArc)
		case revolvecontact.PairSourceCandidate:
			require.Positive(t, len(pair.SourceCells)+len(pair.VerticalCells))
			for _, cell := range pair.SourceCells {
				require.Equal(t, pair.TrimmedFace, cell.Face)
			}
			for _, cell := range pair.VerticalCells {
				require.Equal(t, pair.TrimmedFace, cell.Face)
				require.NotEmpty(t, cell.Retained)
			}
		case revolvecontact.PairToolArc:
			require.NotNil(t, pair.ToolArc)
			require.Equal(t, pair.TrimmedFace, proof.HeelFace)
			require.Equal(t, pair.BlankFace, pair.ToolArc.Circle.BlankFace)
			require.Equal(t, pair.TrimmedFace, pair.ToolArc.Circle.CapFace)
		default:
			t.Fatalf("uncertified face pair verdict %d", pair.Verdict)
		}
	}
	for _, blankFace := range blank.Faces() {
		for _, trimmedFace := range trimmed.Faces() {
			_, found := seen[[2]*Face{blankFace, trimmedFace}]
			require.True(t, found)
		}
	}
	require.Equal(t, 0, counts[revolvecontact.PairRefusal])
	require.Equal(t, 2, counts[revolvecontact.PairToolArc])
	require.Positive(t, counts[revolvecontact.PairSourceCandidate])
	_, err = Union(t.Context(), blank, trimmed)
	require.ErrorIs(t, err, ErrUnsupported,
		"certified face pairs still need a held seam and volume proof")
}

func TestCurvedSourcePairCensusRefusesUnownedRetainedFace(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	shell := trimmed.Shells()[0]
	original := shell.n.Faces
	t.Cleanup(func() { shell.n.Faces = original })
	shell.n.Faces = append(slices.Clone(original), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unowned source face"}}, Body: bodyNode(trimmed)}})))
	_, certified, err := revolvecontact.Census(t.Context(), proof)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, certified,
		"every live retained face needs an authenticated source role")
}

// The census keeps a private deep copy of each arc it derived, and a consumer
// holding the census compares an owned arc with that copy instead of
// rederiving it. This test holds the comparison to the full live check for
// seven sample changes of an owned arc, and requires the comparison with the
// snapshot to refuse a change of each of the 19 rational leaves (in place and
// by replacement) and of every scalar and identity field the arc equality
// reads.
func TestCurvedCensusOwnedArcsMatchLiveCheck(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, census.OwnArcs, 2)
	for _, own := range census.OwnArcs {
		owners := 0
		for i := range census.Pairs {
			if census.Pairs[i].ToolArc == own.Arc {
				owners++
			}
		}
		require.Equal(t, 1, owners)
		require.NotSame(t, own.Arc.Time, own.Derived.Time)
	}

	other := trimmed.Faces()[0]
	type change struct {
		name    string
		apply   func(a *revolvecontact.ToolCapOccupiedArc) (restore func())
		refused bool
	}
	changes := []change{
		{"unchanged", func(*revolvecontact.ToolCapOccupiedArc) func() { return func() {} }, false},
		{"time replaced", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			saved := a.Time
			a.Time = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.Time = saved }
		}, true},
		{"time in place", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			saved := new(big.Rat).Set(a.Time)
			a.Time.Add(a.Time, big.NewRat(1, 1<<20))
			return func() { a.Time.Set(saved) }
		}, true},
		{"parameter replaced", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			saved := a.NegativeZ.Parameter.Lo
			a.NegativeZ.Parameter.Lo = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.NegativeZ.Parameter.Lo = saved }
		}, true},
		{"box in place", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			x := a.PositiveZ.Box[1].Hi
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"radius in place", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			x := a.Circle.Radius
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"through flipped", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			a.ThroughPositiveY = !a.ThroughPositiveY
			return func() { a.ThroughPositiveY = !a.ThroughPositiveY }
		}, true},
		{"face replaced", func(a *revolvecontact.ToolCapOccupiedArc) func() {
			saved := a.NegativeZ.Face
			a.NegativeZ.Face = faceNode(other)
			if faceNode(other) == saved {
				a.NegativeZ.Face = faceNode(trimmed.Faces()[1])
			}
			return func() { a.NegativeZ.Face = saved }
		}, true},
	}
	both := func(c *revolvecontact.PairCensus, arc *revolvecontact.ToolCapOccupiedArc) (bool, bool) {
		in, inErr := revolvecontact.ToolCapOccupiedArcCurrentIn(t.Context(), proof, c, arc)
		full, fullErr := revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.Equal(t, fullErr, inErr)
		require.Equal(t, full, in)
		return in, full
	}
	for _, own := range census.OwnArcs {
		for _, c := range changes {
			restore := c.apply(own.Arc)
			got, _ := both(census, own.Arc)
			require.Equal(t, !c.refused, got, c.name)
			both(nil, own.Arc)
			restore()
		}
		got, _ := both(census, own.Arc)
		require.True(t, got)

		// Every field the equality reads must refuse a change through the
		// snapshot comparison. The full check agrees on the samples above.
		refuses := func(name string, apply func() (restore func())) {
			restore := apply()
			in, err := revolvecontact.ToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.Arc)
			require.NoError(t, err, name)
			require.False(t, in, name)
			restore()
			in, err = revolvecontact.ToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.Arc)
			require.NoError(t, err, name)
			require.True(t, in, name+" restored")
		}
		delta := big.NewRat(1, 1<<20)
		leaves := ratLeaves(own.Arc)
		require.Len(t, leaves, 19)
		for i, leaf := range leaves {
			refuses(fmt.Sprintf("leaf %d in place", i), func() func() {
				saved := new(big.Rat).Set(*leaf)
				(*leaf).Add(*leaf, delta)
				return func() { (*leaf).Set(saved) }
			})
			refuses(fmt.Sprintf("leaf %d replaced", i), func() func() {
				saved := *leaf
				*leaf = new(big.Rat).Add(saved, delta)
				return func() { *leaf = saved }
			})
		}
		a := own.Arc
		faces := trimmed.Faces()
		otherFace := func(f *Face) *Face {
			if f == faces[0] {
				return faces[1]
			}
			return faces[0]
		}
		scalars := map[string]func() func(){
			"band": func() func() { a.Band++; return func() { a.Band-- } },
			"through": func() func() {
				a.ThroughPositiveY = !a.ThroughPositiveY
				return func() { a.ThroughPositiveY = !a.ThroughPositiveY }
			},
			"blankFace": func() func() {
				f := a.Circle.BlankFace
				a.Circle.BlankFace = faceNode(otherFace(faceHandle(f)))
				return func() { a.Circle.BlankFace = f }
			},
			"capFace": func() func() {
				f := a.Circle.CapFace
				a.Circle.CapFace = faceNode(otherFace(faceHandle(f)))
				return func() { a.Circle.CapFace = f }
			},
			"capOrigin": func() func() {
				f := a.Circle.CapOrigin
				a.Circle.CapOrigin.Role += "x"
				return func() { a.Circle.CapOrigin = f }
			},
			"circle span": func() func() { a.Circle.Span++; return func() { a.Circle.Span-- } },
			"tool":        func() func() { a.Circle.Tool++; return func() { a.Circle.Tool-- } },
			"sourceMiss": func() func() {
				a.Circle.SourceMiss = !a.Circle.SourceMiss
				return func() { a.Circle.SourceMiss = !a.Circle.SourceMiss }
			},
		}
		for name, apply := range scalars {
			refuses(name, apply)
		}
		for bi, b := range [...]*revolvecontact.ToolCapBoundary{&a.NegativeZ, &a.PositiveZ} {
			refuses(fmt.Sprintf("boundary %d segment", bi), func() func() { b.Segment++; return func() { b.Segment-- } })
			refuses(fmt.Sprintf("boundary %d span", bi), func() func() { b.Span++; return func() { b.Span-- } })
			refuses(fmt.Sprintf("boundary %d outward", bi), func() func() { b.Outward = !b.Outward; return func() { b.Outward = !b.Outward } })
			refuses(fmt.Sprintf("boundary %d ref", bi), func() func() {
				f := b.Ref
				b.Ref.Role += "x"
				return func() { b.Ref = f }
			})
			refuses(fmt.Sprintf("boundary %d face", bi), func() func() {
				f := b.Face
				b.Face = faceNode(otherFace(faceHandle(f)))
				return func() { b.Face = f }
			})
		}

		// A deep copy is not owned and takes the full path.
		clone := revolvecontact.CloneToolCapOccupiedArc(own.Arc)
		got, _ = both(census, &clone)
		require.True(t, got)
		clone.Time.Add(clone.Time, big.NewRat(1, 1<<20))
		got, _ = both(census, &clone)
		require.False(t, got)
	}

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	for _, own := range census.OwnArcs {
		_, inErr := revolvecontact.ToolCapOccupiedArcCurrentIn(cancelled, proof, census, own.Arc)
		_, fullErr := revolvecontact.ToolCapOccupiedArcCurrent(cancelled, proof, own.Arc)
		require.ErrorIs(t, inErr, context.Canceled)
		require.ErrorIs(t, fullErr, context.Canceled)
	}

	require.Equal(t, 6, reflect.TypeFor[revolvecontact.ToolCapOccupiedArc]().NumField(),
		"update revolvecontact.CloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 8, reflect.TypeFor[revolvecontact.ToolCapCircle]().NumField(),
		"update revolvecontact.CloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 7, reflect.TypeFor[revolvecontact.ToolCapBoundary]().NumField(),
		"update revolvecontact.CloneToolCapOccupiedArc and the Equal functions")
}

// ratLeaves returns the address of every rational an occupied arc holds: the
// circle's x and radius, the time, and each boundary's parameter and box
// bounds.
func ratLeaves(a *revolvecontact.ToolCapOccupiedArc) []**big.Rat {
	out := []**big.Rat{&a.Circle.X, &a.Circle.Radius, &a.Time}
	for _, b := range [...]*revolvecontact.ToolCapBoundary{&a.NegativeZ, &a.PositiveZ} {
		out = append(out, &b.Parameter.Lo, &b.Parameter.Hi)
		for i := range b.Box {
			out = append(out, &b.Box[i].Lo, &b.Box[i].Hi)
		}
	}
	return out
}

func TestCurvedCallScopeSharesLiveRecords(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	ctx := revolvecontact.WithCallScope(t.Context())
	p1, g1, ok, err := revolvecontact.LiveGraph(ctx, bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	p2, g2, ok, err := revolvecontact.LiveGraph(ctx, bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, p1, p2)
	require.Same(t, g1, g2)

	c1, ok, err := revolvecontact.CensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	c2, ok, err := revolvecontact.CensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, c1, c2)

	plain := t.Context()
	u1, ug1, ok, err := revolvecontact.LiveGraph(plain, bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	u2, ug2, ok, err := revolvecontact.LiveGraph(plain, bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, u1, u2)
	require.NotSame(t, ug1, ug2)

	require.Same(t, revolvecontact.CallScopeOf(ctx),
		revolvecontact.CallScopeOf(revolvecontact.WithCallScope(ctx)))
	require.Nil(t, revolvecontact.CallScopeOf(plain))
}

// unscopedAllRunsShell is revolvecontact.BlankShellWithAllRuns without the
// call scope, so every nested check certifies and builds its own records.
func unscopedAllRunsShell(ctx context.Context, proof *revolvecontact.SourceCertificate,
) (*revolvecontact.BlankSeamShell, *revolvecontact.ContactGraph, bool, error) {
	graph, ok, err := revolvecontact.ContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.Runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]revolvecontact.ShellSeamSelector, 0, len(graph.Runs))
	for _, run := range graph.Runs {
		if run.CellID < 0 || run.CellID >= len(graph.Cells) {
			return nil, nil, false, nil
		}
		cell := graph.Cells[run.CellID]
		if cell.Source == nil || cell.PairID < 0 ||
			graph.Pairs[cell.PairID].Verdict != revolvecontact.PairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, revolvecontact.ShellSeamSelector{
			SectionIndex: cell.SectionIndex, CellIndex: cell.CellIndex})
	}
	capCuts, valid := revolvecontact.GraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := revolvecontact.BlankShellAtRunsWithCuts(ctx, proof,
		graph.Manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.FaceBound >= 0.1 ||
		len(shell.Seams) != len(graph.Runs) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) {
		return nil, nil, false, err
	}
	shell.SeamRunIDs = make([][]int, len(graph.Runs))
	for i := range graph.Runs {
		view := *shell
		view.Seam = shell.Seams[i]
		bound, bindErr := graph.BindBlankShell(ctx, proof, &graph.Runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.SeamRunIDs[i] = view.SeamIDs
	}
	shell.SeamIDs = shell.SeamRunIDs[0]
	return shell, graph, true, nil
}

func TestCurvedCallScopeShellMatchesUnscoped(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	shell, graph, ok, err := revolvecontact.BlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ref, refGraph, ok, err := unscopedAllRunsShell(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	require.Equal(t, ref.Mesh.Verts, shell.Mesh.Verts)
	require.Equal(t, ref.Mesh.Tris, shell.Mesh.Tris)
	require.Equal(t, ref.VertexBounds, shell.VertexBounds)
	require.Equal(t, ref.Seams, shell.Seams)
	require.Equal(t, ref.SeamRunIDs, shell.SeamRunIDs)
	require.Equal(t, ref.SeamIDs, shell.SeamIDs)
	require.Equal(t, ref.ChordBound, shell.ChordBound)
	require.Equal(t, ref.FaceBound, shell.FaceBound)
	require.Equal(t, ref.TriangleCells, shell.TriangleCells)
	require.NotEmpty(t, shell.CapRings)
	require.Len(t, shell.CapRings, len(ref.CapRings))
	for i, ring := range shell.CapRings {
		want := ref.CapRings[i]
		require.Equal(t, want.EndpointGraphIDs, ring.EndpointGraphIDs)
		require.Equal(t, want.EndpointCellIDs, ring.EndpointCellIDs)
		require.Equal(t, want.Endpoints, ring.Endpoints)
		require.Equal(t, want.Vertices, ring.Vertices)
	}

	require.Len(t, graph.Pairs, len(refGraph.Pairs))
	require.Len(t, graph.Cells, len(refGraph.Cells))
	require.Len(t, graph.Arcs, len(refGraph.Arcs))
	require.Len(t, graph.Runs, len(refGraph.Runs))
	for i, run := range graph.Runs {
		require.Equal(t, refGraph.Runs[i].CellID, run.CellID)
		require.Equal(t, refGraph.Runs[i].Stations, run.Stations)
		require.Equal(t, refGraph.Runs[i].StationIDs, run.StationIDs)
	}
	for i, arc := range graph.Arcs {
		require.Equal(t, refGraph.Arcs[i].EndpointIDs, arc.EndpointIDs)
		require.Equal(t, refGraph.Arcs[i].EndpointCellIDs, arc.EndpointCellIDs)
	}
	for i, pair := range graph.Pairs {
		require.Equal(t, refGraph.Pairs[i].Verdict, pair.Verdict)
		require.Equal(t, refGraph.Pairs[i].CellIDs, pair.CellIDs)
	}
	same, err := revolvecontact.ContactGraphCurrent(t.Context(), refGraph)
	require.NoError(t, err)
	require.True(t, same)
}

func TestCurvedCallScopeStillRefusesChangedGraph(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolvecontact.ContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ctx := revolvecontact.WithCallScope(t.Context())
	_, scoped, ok, err := revolvecontact.LiveGraph(ctx, bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, graph, scoped)
	current := func() bool {
		live, liveErr := revolvecontact.ContactGraphCurrent(ctx, graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.Cells
		graph.Cells = original[: len(original)-1 : len(original)-1]
		require.False(t, current())
		graph.Cells = original
	})
	t.Run("duplicate endpoint owner", func(t *testing.T) {
		original := graph.Arcs[0].EndpointCellIDs
		graph.Arcs[0].EndpointCellIDs[1] = original[0]
		require.False(t, current())
		graph.Arcs[0].EndpointCellIDs = original
	})
	t.Run("cross-kind event ID", func(t *testing.T) {
		original := graph.Arcs[0].EndpointIDs
		graph.Arcs[0].EndpointIDs[0] = graph.Runs[0].StationIDs[0]
		require.False(t, current())
		graph.Arcs[0].EndpointIDs = original
	})
	t.Run("swapped face", func(t *testing.T) {
		original := graph.Cells[0].TrimmedFace
		graph.Cells[0].TrimmedFace = graph.Pairs[0].TrimmedFace
		if graph.Cells[0].TrimmedFace == original {
			graph.Cells[0].TrimmedFace = graph.Pairs[1].TrimmedFace
		}
		require.False(t, current())
		graph.Cells[0].TrimmedFace = original
	})
	t.Run("changed source ref", func(t *testing.T) {
		original := graph.Cells[0].Source.Ref
		graph.Cells[0].Source.Ref = provenance.Ref{Role: "changed graph source"}
		require.False(t, current())
		graph.Cells[0].Source.Ref = original
	})
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.Arcs[0].Arc.NegativeZ.Parameter.Lo
		graph.Arcs[0].Arc.NegativeZ.Parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.Arcs[0].Arc.NegativeZ.Parameter.Lo = original
	})
	require.True(t, current())
}

func TestCurvedLoftFacesArcSeamPatchesShareCertifiedVertices(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
	seam, blankPatch, loftPatch, ok, err := revolvecontact.CertifiedArcPatches(t.Context(),
		bodyNode(blank), bodyNode(trimmed), sectionIndex, contact.Segment, contact.Band,
		loftFaceInterval{Lo: contact.TLo, Hi: contact.THi}, stations)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, blankPatch.Triangles, 4*(len(stations)-1))
	require.Equal(t, len(blankPatch.Triangles), len(loftPatch.Triangles))
	maxBlank, maxLoft := 0.0, 0.0
	for index := range seam {
		require.Equal(t, blankPatch.Vertices[blankPatch.Seam[index]],
			loftPatch.Vertices[loftPatch.Seam[index]])
		require.Less(t, seam[index].BlankBound, 0.1)
		require.Less(t, seam[index].LoftBound, 0.1)
		maxBlank = max(maxBlank, seam[index].BlankBound)
		maxLoft = max(maxLoft, seam[index].LoftBound)
	}
	for _, bound := range blankPatch.VertexBounds {
		require.Less(t, bound, 0.1)
	}
	for _, bound := range loftPatch.VertexBounds {
		require.Less(t, bound, 0.1)
	}
	t.Logf("source-aligned radial Seam: %d stations, maximum vertex bounds blank %.9g mm, loft %.9g mm",
		len(seam), maxBlank, maxLoft)
}

func neutralArcSeamContact(t *testing.T, blank, trimmed *Body, trim *loftFacesTrimRecord) (
	int, revolveloftfaces.RevolveLoftFacesMappedCell, []int,
) {
	t.Helper()
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolvecontact.Census(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	var section revolveSectionSpan
	sectionIndex := -1
	var contact revolveloftfaces.RevolveLoftFacesMappedCell
	found := false
	for _, pair := range census.Pairs {
		if pair.Verdict != revolvecontact.PairSourceCandidate {
			continue
		}
		for _, cell := range pair.SourceCells {
			mapped := cell.Mapped
			if mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
				mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact || found {
				continue
			}
			section = proof.Section.Spans[pair.SectionIndex]
			sectionIndex = pair.SectionIndex
			contact = mapped
			found = true
		}
	}
	require.True(t, found)
	require.NotNil(t, section.Face)
	require.NotNil(t, contact.JointArc)
	require.NotNil(t, contact.TLo)
	require.NotNil(t, contact.THi)
	var stations []int
	for index, group := range trim.StationGroup {
		if group == contact.Segment {
			stations = append(stations, index)
		}
	}
	require.NotEmpty(t, stations)
	stations = append(stations, (stations[len(stations)-1]+1)%len(trim.StationGroup))
	return sectionIndex, contact, stations
}

func TestCurvedArcSeamRefusesChangedStationAndIncompleteRun(t *testing.T) {
	t.Run("changed station", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		trim.Source.SourceStation[stations[1]].U += 0.001
		require.False(t, facetedrecord.PointSourceCurrent(trim.Source))
		_, _, _, ok, err := revolvecontact.CertifiedArcPatches(t.Context(), bodyNode(blank), bodyNode(trimmed),
			sectionIndex, contact.Segment, contact.Band,
			loftFaceInterval{Lo: contact.TLo, Hi: contact.THi}, stations)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("changed station bound", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		require.Positive(t, trim.Source.SourceStationRound)
		trim.Source.SourceStationRound = 0
		require.False(t, facetedrecord.PointSourceCurrent(trim.Source))
		_, _, _, ok, err := revolvecontact.CertifiedArcPatches(t.Context(), bodyNode(blank), bodyNode(trimmed),
			sectionIndex, contact.Segment, contact.Band,
			loftFaceInterval{Lo: contact.TLo, Hi: contact.THi}, stations)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("truncated arc run", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		require.Greater(t, len(stations), 2)
		_, _, _, ok, err := revolvecontact.CertifiedArcPatches(t.Context(), bodyNode(blank), bodyNode(trimmed),
			sectionIndex, contact.Segment, contact.Band,
			loftFaceInterval{Lo: contact.TLo, Hi: contact.THi}, stations[:2])
		require.NoError(t, err)
		require.False(t, ok)
	})
}

func TestCurvedLoftFacesRealCopySeparation(t *testing.T) {
	_, source, trim := neutralArcSeamSource(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	image, err := Copy(t.Context(), source, WithCopyPlacement(turn))
	require.NoError(t, err)
	original := source.n.Payload.(facetedPayload)
	placed := image.n.Payload.(facetedPayload)
	require.Same(t, trim, original.LoftFacesTrim)
	require.NotNil(t, placed.CopyLineage)
	require.Same(t, source, bodyHandle(placed.CopyLineage.Source))
	require.Equal(t, turn, placed.CopyLineage.Motion)

	shell := func(p facetedPayload) shellseparation.Shell {
		return shellseparation.Shell{Verts: p.Verts, Tris: p.Tris,
			VertexBound: p.VertexBound, MeshBound: p.MeshBound}
	}
	moves := []r3.Transform{r3.Identity(), turn}
	require.True(t, shellseparation.Separate(t.Context(),
		[]shellseparation.Shell{shell(original), shell(placed)}, moves,
		nil, shellseparation.SourceShell,
		shellseparation.AxisX, true))
	held, err := shellseparation.PlacedHeldShells(t.Context(), []shellseparation.HeldCopy{
		{Verts: original.Verts, Tris: original.Tris, Linked: true},
		{Verts: placed.Verts, Tris: placed.Tris, Linked: true},
	}, moves)
	require.NoError(t, err)
	require.True(t, shellseparation.Separate(t.Context(), held, moves,
		nil, shellseparation.HeldCopyShell,
		shellseparation.AxisX, true))

	overlap, err := Copy(t.Context(), source)
	require.NoError(t, err)
	require.False(t, shellseparation.Separate(t.Context(), []shellseparation.Shell{
		shell(original), shell(overlap.n.Payload.(facetedPayload)),
	}, []r3.Transform{r3.Identity(), r3.Identity()}, nil, shellseparation.SourceShell,
		shellseparation.AxisX, true))
}
