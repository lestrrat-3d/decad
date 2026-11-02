package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSameMotherSurfaceMatchesExactCylinderCarriers(t *testing.T) {
	t.Parallel()

	first := surfacegeom.Cylinder{
		Origin: r3.NewVec(2, -1, 4), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(2.7),
	}
	shiftedAlongAxis := surfacegeom.Cylinder{
		Origin: r3.NewVec(2, -1, 9), Axis: r3.NewVec(0, 0, -1), Radius: units.Millimeters(2.7),
	}
	transverseUlp := surfacegeom.Cylinder{
		Origin: r3.NewVec(2, math.Nextafter(-1, math.Inf(1)), 9),
		Axis:   r3.NewVec(0, 0, 1), Radius: units.Millimeters(2.7),
	}
	differentRadius := surfacegeom.Cylinder{
		Origin: first.Origin, Axis: first.Axis,
		Radius: units.Millimeters(math.Nextafter(2.7, math.Inf(1))),
	}

	require.True(t, sameMotherSurface(first, shiftedAlongAxis))
	require.False(t, sameMotherSurface(first, transverseUlp))
	require.False(t, sameMotherSurface(first, differentRadius))
	require.False(t, sameMotherSurface(first, surfacegeom.Cylinder{}))
}

func TestExactMotherAxisLineUsesRationalCrossProducts(t *testing.T) {
	t.Parallel()

	axis := r3.NewVec(1, 2, 3)
	require.True(t, exactMotherAxisLine(r3.Vec{}, r3.NewVec(4, 8, 12), axis))
	require.False(t, exactMotherAxisLine(r3.Vec{}, r3.NewVec(4, 8, math.Nextafter(12, math.Inf(1))), axis))
}

func TestRemapMotherAnalyticCutFacesUsesUniqueRolesAndSources(t *testing.T) {
	t.Parallel()

	knuckle, bore, adjustedFace, boreFace, cutFaces := motherCutLineageFixture(t, false)
	adjustedLineage := motherCutAdjustedLineage(knuckle, adjustedFace, cutFaces)
	lineage, ok := remapMotherAnalyticCutFaces(cutFaces, 30,
		adjustedLineage,
		[]motherFaceLineage{{sourceFace: boreFace, rootBody: bore, rootFace: boreFace, rootOrdinal: 0}})
	require.True(t, ok)
	require.Len(t, lineage, 4)
	require.Same(t, knuckle, lineage[0].rootBody)
	require.Equal(t, 0, lineage[0].rootOrdinal)
	require.Equal(t, provenance.Producer(1), lineage[0].sourceFace.Source.Producer)
	require.Equal(t, 0, lineage[0].sourceFace.Source.Operand)
	require.Same(t, adjustedFace.Source, lineage[0].sourceFace.Source.Parent)
	require.Same(t, bore, lineage[1].rootBody)
	require.Equal(t, 1, lineage[1].sourceFace.Source.Operand)
	require.Same(t, boreFace.Source, lineage[1].sourceFace.Source.Parent)
}

func TestRemapMotherAnalyticCutFacesRejectsUnmatchedAndDuplicateOwners(t *testing.T) {
	t.Parallel()

	knuckle, bore, adjustedFace, boreFace, cutFaces := motherCutLineageFixture(t, true)
	adjusted := motherCutAdjustedLineage(knuckle, adjustedFace, cutFaces)
	tool := []motherFaceLineage{{sourceFace: boreFace, rootBody: bore, rootFace: boreFace, rootOrdinal: 0}}
	_, ok := remapMotherAnalyticCutFaces(cutFaces, 30, adjusted, tool)
	require.False(t, ok)

	knuckle, bore, adjustedFace, boreFace, duplicateFaces := motherCutLineageFixture(t, false)
	adjusted = motherCutAdjustedLineage(knuckle, adjustedFace, duplicateFaces)
	tool = []motherFaceLineage{{sourceFace: boreFace, rootBody: bore, rootFace: boreFace, rootOrdinal: 0}}
	duplicateFaces[1].Origins[0].Role = duplicateFaces[0].Origins[0].Role
	_, ok = remapMotherAnalyticCutFaces(duplicateFaces, 30, adjusted, tool)
	require.False(t, ok)
}

func TestMotherHingeUnionWarningNamesPublicCutTarget(t *testing.T) {
	t.Parallel()

	body := bornBody(&Body{n: topo.Body{OperationWarnings: []opdiagnostic.Record{{
		Operation: int(meshbool.OpUnion), Code: opdiagnostic.CodeGapShiftApplied,
		PublicOperation: 0, Operand: 1, Change: opdiagnostic.ChangeGapShift,
		DisplacementBound: .001, AllowedReach: .01,
	}}}})
	require.True(t, remapMotherHingeUnionWarningForPublicCut(body))
	stampOperationWarningKind(body, OperationCut)
	diagnostics := body.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OperationCut, diagnostics[0].Operation)
	require.Equal(t, "target", diagnostics[0].Subject.Name)
	require.Equal(t, OperationChangeGapShift, diagnostics[0].Change.Mechanism)
}

func motherCutLineageFixture(t *testing.T, moveBoreTransversely bool) (*Body, *Body, *topo.Face, *topo.Face, []*topo.Face) {
	t.Helper()
	frameAt := func(z float64) r3.Frame {
		frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		return frame
	}
	knuckleBody := bornBody(&Body{n: topo.Body{Origin: provenance.Ref{Producer: 1}}})
	boreBody := bornBody(&Body{n: topo.Body{Origin: provenance.Ref{Producer: 2}}})
	outer := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(5)}
	innerOrigin := r3.Vec{}
	if moveBoreTransversely {
		innerOrigin.X = math.SmallestNonzeroFloat64
	}
	inner := surfacegeom.Cylinder{Origin: innerOrigin, Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(2.7)}
	knuckleSource := &topo.Face{Surface: outer, Source: &provenance.SourcePatch{Producer: 1, Ordinal: 0}}
	boreSource := &topo.Face{Surface: inner, Source: &provenance.SourcePatch{Producer: 2, Ordinal: 0}}
	adjustedSource := &topo.Face{Surface: outer,
		Source: &provenance.SourcePatch{Producer: 20, Operand: 1, Ordinal: 0, Parent: knuckleSource.Source}}
	cutOuter := &topo.Face{Surface: outer, Origins: []provenance.Ref{{Producer: 30, Role: "side(0,0)"}}}
	cutInner := &topo.Face{Surface: surfacegeom.Cylinder{
		Origin: r3.NewVec(0, 0, 15), Axis: r3.NewVec(0, 0, -1), Radius: units.Millimeters(2.7)},
		Origins: []provenance.Ref{{Producer: 30, Role: "side(1,0)"}}}
	cutStart := &topo.Face{Surface: surfacegeom.Plane{Frame: frameAt(0)},
		Origins: []provenance.Ref{{Producer: 30, Role: "capStart"}}}
	cutEnd := &topo.Face{Surface: surfacegeom.Plane{Frame: frameAt(15)},
		Origins: []provenance.Ref{{Producer: 30, Role: "capEnd"}}}
	return knuckleBody, boreBody, adjustedSource, boreSource, []*topo.Face{cutOuter, cutInner, cutStart, cutEnd}
}

func motherCutAdjustedLineage(knuckle *Body, adjustedFace *topo.Face, cutFaces []*topo.Face) []motherFaceLineage {
	lineage := []motherFaceLineage{{sourceFace: adjustedFace, rootBody: knuckle,
		rootFace: adjustedFace, rootOrdinal: 0}}
	for i, capFace := range cutFaces[2:] {
		face := &topo.Face{Surface: capFace.Surface,
			Source: &provenance.SourcePatch{Producer: 1, Operand: 1, Ordinal: i + 1}}
		lineage = append(lineage, motherFaceLineage{sourceFace: face, rootBody: knuckle,
			rootFace: face, rootOrdinal: i + 1})
	}
	return lineage
}
