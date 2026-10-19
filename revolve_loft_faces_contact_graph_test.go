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
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCurvedContactGraphHeldAnchors(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.pairs, len(blank.Faces())*len(trimmed.Faces()))
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.pairs, len(census.pairs))
	for i, pair := range census.pairs {
		require.Equal(t, pair.verdict, graph.pairs[i].verdict)
		require.Equal(t, pair.blankFace, graph.pairs[i].blankFace)
		require.Equal(t, pair.trimmedFace, graph.pairs[i].trimmedFace)
		require.Len(t, graph.pairs[i].cellIDs,
			len(pair.sourceCells)+len(pair.verticalCells))
	}
	expectedCells := 0
	for _, side := range census.manifest.nonvertical {
		expectedCells += len(side.cells)
	}
	for _, side := range census.manifest.vertical {
		if side.side != nil {
			expectedCells += len(side.side.cells)
		}
	}
	require.Len(t, graph.cells, expectedCells)
	seenCells := make(map[int]bool)
	for pairID, pair := range graph.pairs {
		for _, cellID := range pair.cellIDs {
			require.False(t, seenCells[cellID])
			seenCells[cellID] = true
			require.Equal(t, pairID, graph.cells[cellID].pairID)
			require.Equal(t, pair.blankFace, graph.cells[cellID].blankFace)
			require.Equal(t, pair.trimmedFace, graph.cells[cellID].trimmedFace)
		}
	}
	for cellID, cell := range graph.cells {
		if cell.trimmedFace == nil {
			require.Equal(t, -1, cell.pairID)
			require.False(t, seenCells[cellID])
		} else {
			require.True(t, seenCells[cellID])
		}
	}
	require.Len(t, graph.runs, 2)
	seenEventIDs := make(map[int]bool)
	for _, arc := range graph.arcs {
		for _, id := range arc.endpointIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
	}
	seenStationIDs := make(map[int]bool)
	for _, run := range graph.runs {
		for _, id := range run.stationIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
		cell := graph.cells[run.cellID]
		require.NotNil(t, cell.source)
		mapped := cell.source.mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolveLoftFacesCertifiedArcPatches(t.Context(),
			blank, trimmed, cell.sectionIndex, mapped.Segment, mapped.Band,
			loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, run.stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		shell, built, shellErr := revolveLoftFacesBlankShellWithCapRings(t.Context(), proof,
			cell.sectionIndex, cell.cellIndex)
		require.NoError(t, shellErr)
		require.True(t, built)
		require.Equal(t, run.stationIDs, shell.seamIDs)
		require.Len(t, run.stationIDs, len(run.stations))
		seenShellVertices := make(map[int]bool)
		for i, id := range run.stationIDs {
			require.False(t, seenStationIDs[id])
			seenStationIDs[id] = true
			shellIndex, blankIndex, loftIndex := shell.seam[i], blankPatch.Seam[i], loftPatch.Seam[i]
			require.False(t, seenShellVertices[shellIndex])
			seenShellVertices[shellIndex] = true
			require.Equal(t, seam[i].Point, shell.mesh.Verts[shellIndex])
			require.Equal(t, seam[i].Point, blankPatch.Vertices[blankIndex])
			require.Equal(t, seam[i].Point, loftPatch.Vertices[loftIndex])
			require.GreaterOrEqual(t, shell.vertexBounds[shellIndex], seam[i].BlankBound)
			require.GreaterOrEqual(t, blankPatch.VertexBounds[blankIndex], seam[i].BlankBound)
			require.GreaterOrEqual(t, loftPatch.VertexBounds[loftIndex], seam[i].LoftBound)
		}
		require.Len(t, shell.capRings, len(graph.arcs))
		seenEndpointIDs, seenRings := make(map[int]bool), make(map[int]bool)
		for _, arc := range graph.arcs {
			matched := -1
			for i, ring := range shell.capRings {
				if revolveLoftFacesToolCapOccupiedArcEqual(arc.arc, ring.arc) {
					require.Equal(t, -1, matched)
					matched = i
				}
			}
			require.GreaterOrEqual(t, matched, 0)
			require.False(t, seenRings[matched])
			seenRings[matched] = true
			ring := shell.capRings[matched]
			require.Equal(t, arc.endpointIDs, ring.endpointGraphIDs)
			require.Equal(t, arc.endpointCellIDs, ring.endpointCellIDs)
			for endpoint, cellID := range arc.endpointCellIDs {
				id := arc.endpointIDs[endpoint]
				require.False(t, seenEndpointIDs[id])
				seenEndpointIDs[id] = true
				require.Equal(t, ring.endpointProof[endpoint].point,
					shell.mesh.Verts[ring.endpoints[endpoint]])
				require.GreaterOrEqual(t, graph.cells[cellID].pairID, 0)
				require.True(t, graph.endpointCellMatches(arc, &ring, endpoint, cellID))
			}
		}
		require.Len(t, seenRings, len(graph.arcs))
	}
	require.Len(t, seenStationIDs, 36)
	require.Len(t, seenEventIDs, 36+2*len(graph.arcs))
	current, err := revolveLoftFacesContactGraphCurrent(t.Context(), graph)
	require.NoError(t, err)
	require.True(t, current)
}

func TestCurvedContactGraphRejectsStaleSourceEvents(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	current := func() bool {
		live, liveErr := revolveLoftFacesContactGraphCurrent(t.Context(), graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.cells
		graph.cells = slices.Clone(original[:len(original)-1])
		require.False(t, current())
		graph.cells = original
	})
	t.Run("duplicate endpoint owner", func(t *testing.T) {
		original := graph.arcs[0].endpointCellIDs
		graph.arcs[0].endpointCellIDs[1] = original[0]
		require.False(t, current())
		graph.arcs[0].endpointCellIDs = original
	})
	t.Run("cross-kind event ID", func(t *testing.T) {
		original := graph.arcs[0].endpointIDs
		graph.arcs[0].endpointIDs[0] = graph.runs[0].stationIDs[0]
		require.False(t, current())
		graph.arcs[0].endpointIDs = original
	})
	t.Run("swapped face", func(t *testing.T) {
		original := graph.cells[0].trimmedFace
		graph.cells[0].trimmedFace = graph.pairs[0].trimmedFace
		if graph.cells[0].trimmedFace == original {
			graph.cells[0].trimmedFace = graph.pairs[1].trimmedFace
		}
		require.False(t, current())
		graph.cells[0].trimmedFace = original
	})
	t.Run("changed source ref", func(t *testing.T) {
		original := graph.cells[0].source.ref
		graph.cells[0].source.ref = provenance.Ref{Role: "changed graph source"}
		require.False(t, current())
		graph.cells[0].source.ref = original
	})
	t.Run("changed open interval", func(t *testing.T) {
		original := graph.cells[0].source.mapped.TLoOpen
		graph.cells[0].source.mapped.TLoOpen = !original
		require.False(t, current())
		graph.cells[0].source.mapped.TLoOpen = original
	})
	t.Run("changed live station", func(t *testing.T) {
		original := trim.Source.SourceStation[0]
		trim.Source.SourceStation[0].U += 0.001
		require.False(t, current())
		trim.Source.SourceStation[0] = original
	})
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.arcs[0].arc.negativeZ.parameter.Lo
		graph.arcs[0].arc.negativeZ.parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.arcs[0].arc.negativeZ.parameter.Lo = original
	})
	require.True(t, current())
}

func TestCurvedSourcePairCensusCoversLiveFaces(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, census.manifest)
	require.Len(t, census.pairs, len(blank.Faces())*len(trimmed.Faces()))

	seen := make(map[[2]*Face]struct{})
	counts := make(map[revolveLoftFacesPairVerdict]int)
	for _, pair := range census.pairs {
		key := [2]*Face{pair.blankFace, pair.trimmedFace}
		_, duplicate := seen[key]
		require.False(t, duplicate, "a live face pair must occur once")
		seen[key] = struct{}{}
		require.Equal(t, proof.section.Spans[pair.sectionIndex].Face, faceNode(pair.blankFace))
		counts[pair.verdict]++
		switch pair.verdict {
		case revolveLoftFacesPairMiss:
			require.Nil(t, pair.toolArc)
		case revolveLoftFacesPairSourceCandidate:
			require.Positive(t, len(pair.sourceCells)+len(pair.verticalCells))
			for _, cell := range pair.sourceCells {
				require.Equal(t, pair.trimmedFace, cell.face)
			}
			for _, cell := range pair.verticalCells {
				require.Equal(t, pair.trimmedFace, cell.face)
				require.NotEmpty(t, cell.retained)
			}
		case revolveLoftFacesPairToolArc:
			require.NotNil(t, pair.toolArc)
			require.Equal(t, pair.trimmedFace, proof.heelFace)
			require.Equal(t, pair.blankFace, pair.toolArc.circle.blankFace)
			require.Equal(t, pair.trimmedFace, pair.toolArc.circle.capFace)
		default:
			t.Fatalf("uncertified face pair verdict %d", pair.verdict)
		}
	}
	for _, blankFace := range blank.Faces() {
		for _, trimmedFace := range trimmed.Faces() {
			_, found := seen[[2]*Face{blankFace, trimmedFace}]
			require.True(t, found)
		}
	}
	require.Equal(t, 0, counts[revolveLoftFacesPairRefusal])
	require.Equal(t, 2, counts[revolveLoftFacesPairToolArc])
	require.Positive(t, counts[revolveLoftFacesPairSourceCandidate])
	_, err = Union(t.Context(), blank, trimmed)
	require.ErrorIs(t, err, ErrUnsupported,
		"certified face pairs still need a held seam and volume proof")
}

func TestCurvedSourcePairCensusRefusesUnownedRetainedFace(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	shell := trimmed.Shells()[0]
	original := shell.n.Faces
	t.Cleanup(func() { shell.n.Faces = original })
	shell.n.Faces = append(slices.Clone(original), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unowned source face"}}, Body: bodyNode(trimmed)}})))
	_, certified, err := revolveLoftFacesCensus(t.Context(), proof)
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
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, census.ownArcs, 2)
	for _, own := range census.ownArcs {
		owners := 0
		for i := range census.pairs {
			if census.pairs[i].toolArc == own.arc {
				owners++
			}
		}
		require.Equal(t, 1, owners)
		require.NotSame(t, own.arc.time, own.derived.time)
	}

	other := trimmed.Faces()[0]
	type change struct {
		name    string
		apply   func(a *revolveLoftFacesToolCapOccupiedArc) (restore func())
		refused bool
	}
	changes := []change{
		{"unchanged", func(*revolveLoftFacesToolCapOccupiedArc) func() { return func() {} }, false},
		{"time replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.time
			a.time = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.time = saved }
		}, true},
		{"time in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := new(big.Rat).Set(a.time)
			a.time.Add(a.time, big.NewRat(1, 1<<20))
			return func() { a.time.Set(saved) }
		}, true},
		{"parameter replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.negativeZ.parameter.Lo
			a.negativeZ.parameter.Lo = new(big.Rat).Add(saved, big.NewRat(1, 1<<20))
			return func() { a.negativeZ.parameter.Lo = saved }
		}, true},
		{"box in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			x := a.positiveZ.box[1].Hi
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"radius in place", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			x := a.circle.radius
			saved := new(big.Rat).Set(x)
			x.Add(x, big.NewRat(1, 1<<20))
			return func() { x.Set(saved) }
		}, true},
		{"through flipped", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			a.throughPositiveY = !a.throughPositiveY
			return func() { a.throughPositiveY = !a.throughPositiveY }
		}, true},
		{"face replaced", func(a *revolveLoftFacesToolCapOccupiedArc) func() {
			saved := a.negativeZ.face
			a.negativeZ.face = other
			if other == saved {
				a.negativeZ.face = trimmed.Faces()[1]
			}
			return func() { a.negativeZ.face = saved }
		}, true},
	}
	both := func(c *revolveLoftFacesPairCensus, arc *revolveLoftFacesToolCapOccupiedArc) (bool, bool) {
		in, inErr := revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, c, arc)
		full, fullErr := revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.Equal(t, fullErr, inErr)
		require.Equal(t, full, in)
		return in, full
	}
	for _, own := range census.ownArcs {
		for _, c := range changes {
			restore := c.apply(own.arc)
			got, _ := both(census, own.arc)
			require.Equal(t, !c.refused, got, c.name)
			both(nil, own.arc)
			restore()
		}
		got, _ := both(census, own.arc)
		require.True(t, got)

		// Every field the equality reads must refuse a change through the
		// snapshot comparison. The full check agrees on the samples above.
		refuses := func(name string, apply func() (restore func())) {
			restore := apply()
			in, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.arc)
			require.NoError(t, err, name)
			require.False(t, in, name)
			restore()
			in, err = revolveLoftFacesToolCapOccupiedArcCurrentIn(t.Context(), proof, census, own.arc)
			require.NoError(t, err, name)
			require.True(t, in, name+" restored")
		}
		delta := big.NewRat(1, 1<<20)
		leaves := ratLeaves(own.arc)
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
		a := own.arc
		faces := trimmed.Faces()
		otherFace := func(f *Face) *Face {
			if f == faces[0] {
				return faces[1]
			}
			return faces[0]
		}
		scalars := map[string]func() func(){
			"band": func() func() { a.band++; return func() { a.band-- } },
			"through": func() func() {
				a.throughPositiveY = !a.throughPositiveY
				return func() { a.throughPositiveY = !a.throughPositiveY }
			},
			"blankFace": func() func() {
				f := a.circle.blankFace
				a.circle.blankFace = otherFace(f)
				return func() { a.circle.blankFace = f }
			},
			"capFace": func() func() {
				f := a.circle.capFace
				a.circle.capFace = otherFace(f)
				return func() { a.circle.capFace = f }
			},
			"capOrigin": func() func() {
				f := a.circle.capOrigin
				a.circle.capOrigin.Role += "x"
				return func() { a.circle.capOrigin = f }
			},
			"circle span": func() func() { a.circle.span++; return func() { a.circle.span-- } },
			"tool":        func() func() { a.circle.tool++; return func() { a.circle.tool-- } },
			"sourceMiss": func() func() {
				a.circle.sourceMiss = !a.circle.sourceMiss
				return func() { a.circle.sourceMiss = !a.circle.sourceMiss }
			},
		}
		for name, apply := range scalars {
			refuses(name, apply)
		}
		for bi, b := range [...]*revolveLoftFacesToolCapBoundary{&a.negativeZ, &a.positiveZ} {
			refuses(fmt.Sprintf("boundary %d segment", bi), func() func() { b.segment++; return func() { b.segment-- } })
			refuses(fmt.Sprintf("boundary %d span", bi), func() func() { b.span++; return func() { b.span-- } })
			refuses(fmt.Sprintf("boundary %d outward", bi), func() func() { b.outward = !b.outward; return func() { b.outward = !b.outward } })
			refuses(fmt.Sprintf("boundary %d ref", bi), func() func() {
				f := b.ref
				b.ref.Role += "x"
				return func() { b.ref = f }
			})
			refuses(fmt.Sprintf("boundary %d face", bi), func() func() {
				f := b.face
				b.face = otherFace(f)
				return func() { b.face = f }
			})
		}

		// A deep copy is not owned and takes the full path.
		clone := revolveLoftFacesCloneToolCapOccupiedArc(own.arc)
		got, _ = both(census, &clone)
		require.True(t, got)
		clone.time.Add(clone.time, big.NewRat(1, 1<<20))
		got, _ = both(census, &clone)
		require.False(t, got)
	}

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	for _, own := range census.ownArcs {
		_, inErr := revolveLoftFacesToolCapOccupiedArcCurrentIn(cancelled, proof, census, own.arc)
		_, fullErr := revolveLoftFacesToolCapOccupiedArcCurrent(cancelled, proof, own.arc)
		require.ErrorIs(t, inErr, context.Canceled)
		require.ErrorIs(t, fullErr, context.Canceled)
	}

	require.Equal(t, 6, reflect.TypeFor[revolveLoftFacesToolCapOccupiedArc]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 8, reflect.TypeFor[revolveLoftFacesToolCapCircle]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
	require.Equal(t, 7, reflect.TypeFor[revolveLoftFacesToolCapBoundary]().NumField(),
		"update revolveLoftFacesCloneToolCapOccupiedArc and the Equal functions")
}

// ratLeaves returns the address of every rational an occupied arc holds: the
// circle's x and radius, the time, and each boundary's parameter and box
// bounds.
func ratLeaves(a *revolveLoftFacesToolCapOccupiedArc) []**big.Rat {
	out := []**big.Rat{&a.circle.x, &a.circle.radius, &a.time}
	for _, b := range [...]*revolveLoftFacesToolCapBoundary{&a.negativeZ, &a.positiveZ} {
		out = append(out, &b.parameter.Lo, &b.parameter.Hi)
		for i := range b.box {
			out = append(out, &b.box[i].Lo, &b.box[i].Hi)
		}
	}
	return out
}

func TestCurvedCallScopeSharesLiveRecords(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	ctx := revolveLoftFacesWithCallScope(t.Context())
	p1, g1, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	p2, g2, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, p1, p2)
	require.Same(t, g1, g2)

	c1, ok, err := revolveLoftFacesCensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	c2, ok, err := revolveLoftFacesCensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, c1, c2)

	plain := t.Context()
	u1, ug1, ok, err := revolveLoftFacesLiveGraph(plain, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	u2, ug2, ok, err := revolveLoftFacesLiveGraph(plain, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, u1, u2)
	require.NotSame(t, ug1, ug2)

	require.Same(t, revolveLoftFacesCallScopeOf(ctx),
		revolveLoftFacesCallScopeOf(revolveLoftFacesWithCallScope(ctx)))
	require.Nil(t, revolveLoftFacesCallScopeOf(plain))
}

// unscopedAllRunsShell is revolveLoftFacesBlankShellWithAllRuns without the
// call scope, so every nested check certifies and builds its own records.
func unscopedAllRunsShell(ctx context.Context, proof *revolveLoftFacesSourceCertificate,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph, bool, error) {
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]revolveLoftFacesShellSeamSelector, 0, len(graph.runs))
	for _, run := range graph.runs {
		if run.cellID < 0 || run.cellID >= len(graph.cells) {
			return nil, nil, false, nil
		}
		cell := graph.cells[run.cellID]
		if cell.source == nil || cell.pairID < 0 ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, revolveLoftFacesShellSeamSelector{
			sectionIndex: cell.sectionIndex, cellIndex: cell.cellIndex})
	}
	capCuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof,
		graph.manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.faceBound >= 0.1 ||
		len(shell.seams) != len(graph.runs) ||
		len(shell.triangleCells) != len(shell.mesh.Tris) {
		return nil, nil, false, err
	}
	shell.seamRunIDs = make([][]int, len(graph.runs))
	for i := range graph.runs {
		view := *shell
		view.seam = shell.seams[i]
		bound, bindErr := graph.bindBlankShell(ctx, proof, &graph.runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.seamRunIDs[i] = view.seamIDs
	}
	shell.seamIDs = shell.seamRunIDs[0]
	return shell, graph, true, nil
}

func TestCurvedCallScopeShellMatchesUnscoped(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	shell, graph, ok, err := revolveLoftFacesBlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ref, refGraph, ok, err := unscopedAllRunsShell(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	require.Equal(t, ref.mesh.Verts, shell.mesh.Verts)
	require.Equal(t, ref.mesh.Tris, shell.mesh.Tris)
	require.Equal(t, ref.vertexBounds, shell.vertexBounds)
	require.Equal(t, ref.seams, shell.seams)
	require.Equal(t, ref.seamRunIDs, shell.seamRunIDs)
	require.Equal(t, ref.seamIDs, shell.seamIDs)
	require.Equal(t, ref.chordBound, shell.chordBound)
	require.Equal(t, ref.faceBound, shell.faceBound)
	require.Equal(t, ref.triangleCells, shell.triangleCells)
	require.NotEmpty(t, shell.capRings)
	require.Len(t, shell.capRings, len(ref.capRings))
	for i, ring := range shell.capRings {
		want := ref.capRings[i]
		require.Equal(t, want.endpointGraphIDs, ring.endpointGraphIDs)
		require.Equal(t, want.endpointCellIDs, ring.endpointCellIDs)
		require.Equal(t, want.endpoints, ring.endpoints)
		require.Equal(t, want.vertices, ring.vertices)
	}

	require.Len(t, graph.pairs, len(refGraph.pairs))
	require.Len(t, graph.cells, len(refGraph.cells))
	require.Len(t, graph.arcs, len(refGraph.arcs))
	require.Len(t, graph.runs, len(refGraph.runs))
	for i, run := range graph.runs {
		require.Equal(t, refGraph.runs[i].cellID, run.cellID)
		require.Equal(t, refGraph.runs[i].stations, run.stations)
		require.Equal(t, refGraph.runs[i].stationIDs, run.stationIDs)
	}
	for i, arc := range graph.arcs {
		require.Equal(t, refGraph.arcs[i].endpointIDs, arc.endpointIDs)
		require.Equal(t, refGraph.arcs[i].endpointCellIDs, arc.endpointCellIDs)
	}
	for i, pair := range graph.pairs {
		require.Equal(t, refGraph.pairs[i].verdict, pair.verdict)
		require.Equal(t, refGraph.pairs[i].cellIDs, pair.cellIDs)
	}
	same, err := revolveLoftFacesContactGraphCurrent(t.Context(), refGraph)
	require.NoError(t, err)
	require.True(t, same)
}

func TestCurvedCallScopeStillRefusesChangedGraph(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ctx := revolveLoftFacesWithCallScope(t.Context())
	_, scoped, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, graph, scoped)
	current := func() bool {
		live, liveErr := revolveLoftFacesContactGraphCurrent(ctx, graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.cells
		graph.cells = original[: len(original)-1 : len(original)-1]
		require.False(t, current())
		graph.cells = original
	})
	t.Run("duplicate endpoint owner", func(t *testing.T) {
		original := graph.arcs[0].endpointCellIDs
		graph.arcs[0].endpointCellIDs[1] = original[0]
		require.False(t, current())
		graph.arcs[0].endpointCellIDs = original
	})
	t.Run("cross-kind event ID", func(t *testing.T) {
		original := graph.arcs[0].endpointIDs
		graph.arcs[0].endpointIDs[0] = graph.runs[0].stationIDs[0]
		require.False(t, current())
		graph.arcs[0].endpointIDs = original
	})
	t.Run("swapped face", func(t *testing.T) {
		original := graph.cells[0].trimmedFace
		graph.cells[0].trimmedFace = graph.pairs[0].trimmedFace
		if graph.cells[0].trimmedFace == original {
			graph.cells[0].trimmedFace = graph.pairs[1].trimmedFace
		}
		require.False(t, current())
		graph.cells[0].trimmedFace = original
	})
	t.Run("changed source ref", func(t *testing.T) {
		original := graph.cells[0].source.ref
		graph.cells[0].source.ref = provenance.Ref{Role: "changed graph source"}
		require.False(t, current())
		graph.cells[0].source.ref = original
	})
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.arcs[0].arc.negativeZ.parameter.Lo
		graph.arcs[0].arc.negativeZ.parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.arcs[0].arc.negativeZ.parameter.Lo = original
	})
	require.True(t, current())
}

func TestCurvedLoftFacesArcSeamPatchesShareCertifiedVertices(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
	seam, blankPatch, loftPatch, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(),
		blank, trimmed, sectionIndex, contact.Segment, contact.Band,
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
	t.Logf("source-aligned radial seam: %d stations, maximum vertex bounds blank %.9g mm, loft %.9g mm",
		len(seam), maxBlank, maxLoft)
}

func neutralArcSeamContact(t *testing.T, blank, trimmed *Body, trim *loftFacesTrimRecord) (
	int, revolveloftfaces.RevolveLoftFacesMappedCell, []int,
) {
	t.Helper()
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	var section revolveSectionSpan
	sectionIndex := -1
	var contact revolveloftfaces.RevolveLoftFacesMappedCell
	found := false
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		for _, cell := range pair.sourceCells {
			mapped := cell.mapped
			if mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
				mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact || found {
				continue
			}
			section = proof.section.Spans[pair.sectionIndex]
			sectionIndex = pair.sectionIndex
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
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
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
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
			sectionIndex, contact.Segment, contact.Band,
			loftFaceInterval{Lo: contact.TLo, Hi: contact.THi}, stations)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("truncated arc run", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		require.Greater(t, len(stations), 2)
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
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
