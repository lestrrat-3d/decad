package decad

import (
	"fmt"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesVerticalRefusesChangedBlankSection(t *testing.T) {
	blank := sectionSketchBody(t, func(s *sketch.Sketch) {
		section := []planePoint{{U: 2}, {U: 5}, {U: 5, V: 3}, {U: 2, V: 1}}
		points := make([]*sketch.Point, len(section))
		for i, point := range section {
			points[i] = s.CreatePoint(point.U, point.V)
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}, FullRevolution{})
	candidates := revolveshell.RootCandidates(bodyNode(blank))
	require.Len(t, candidates, 1)
	proof := &revolvecontact.SourceCertificate{Blank: bodyNode(blank), Section: candidates[0]}
	require.True(t, revolvecontact.SectionCurrent(proof))
	index := -1
	for i, span := range proof.Section.Spans {
		if span.Start.U == 5 && span.End.U == 5 {
			index = i
			break
		}
	}
	require.NotEqual(t, -1, index)
	proof.Section.Spans[index].Start.U = 2
	proof.Section.Spans[index].End.U = 2
	proof.Section.Spans[index].End.V = 2
	require.False(t, revolvecontact.SectionCurrent(proof),
		"the retained face cannot authenticate changed section endpoints")
}

func TestRevolveLoftFacesConstantXRejectsSubstitutedSourceRef(t *testing.T) {
	const segmentCount = 2
	trim := &loftFacesTrimRecord{
		Source: &pointSectionRecord{Profile: profileRecord{Outer: loopRecord{
			Segments: []curveSegment{lineSeg{}, lineSeg{}},
		}}},
		Maps: make([]loftFaceMap, 3),
	}
	trim.Base.Groups = make([]facetGroup, 2*segmentCount+2)
	bound := make([][]provenance.Ref, 2)
	for band := range bound {
		bound[band] = make([]provenance.Ref, segmentCount)
		for segment := range bound[band] {
			ref := provenance.Ref{Producer: 7, Role: fmt.Sprintf("side(%d,%d)", band, segment)}
			trim.Base.Groups[band*segmentCount+segment].Origins = []provenance.Ref{ref}
			bound[band][segment] = ref
		}
	}
	tooth := bornBody(&Body{})
	require.True(t, revolvecontact.SourceRefsCurrent(trim, bodyNode(tooth), bound))
	bound[0][0] = bound[0][1]
	require.False(t, revolvecontact.SourceRefsCurrent(trim, bodyNode(tooth), bound),
		"a retained cell cannot name the neighboring source segment")
}

func TestRevolveLoftFacesVerticalConstantXAndTrimLimits(t *testing.T) {
	zero, one := lfZero(), lfOne()
	mapped := loftFaceMap{
		Constant: loftFaceRatVec{big.NewRat(5, 1), zero, zero},
		Q0:       loftFaceRatVec{big.NewRat(2, 1), zero, zero},
		QU:       loftFaceRatVec{big.NewRat(6, 1), one, zero},
		QV:       loftFaceRatVec{big.NewRat(4, 1), zero, one},
		G0:       one, GU: big.NewRat(3, 1), GV: big.NewRat(2, 1),
	}
	x, ok := loftface.ConstantX(mapped)
	require.True(t, ok)
	require.Equal(t, "7", x.RatString())
	mapped.QU[0] = big.NewRat(5, 1)
	_, ok = loftface.ConstantX(mapped)
	require.False(t, ok)

	trim := &loftFacesTrimRecord{
		Toe:  &pointConeLimit{Apex: big.NewRat(13, 2), Far: big.NewRat(-2, 1), Slope: one},
		Heel: &pointConeLimit{Apex: big.NewRat(8, 1), Far: big.NewRat(-2, 1), Slope: one},
	}
	section := revolveSectionSpan{Start: planePoint{U: 6}, End: planePoint{U: 6, V: 3.5}}
	lower, upper, outcome := revolveloftfaces.VerticalLimits(trim, section, big.NewRat(6, 1))
	require.Equal(t, revolveloftfaces.VerticalContact, outcome)
	require.Equal(t, "1/2", lower.RatString())
	require.Equal(t, "2", upper.RatString())
	section.Start.U, section.End.U = 3.2, 3.2
	lower, upper, outcome = revolveloftfaces.VerticalLimits(trim, section, big.NewRat(16, 5))
	require.Equal(t, revolveloftfaces.VerticalContact, outcome)
	require.Equal(t, 0, lower.Cmp(big.NewRat(33, 10)))
	require.Equal(t, "7/2", upper.RatString())
	section.End.V = 3.2
	lower, upper, outcome = revolveloftfaces.VerticalLimits(trim, section, lfF(3.2))
	require.Equal(t, revolveloftfaces.VerticalMiss, outcome)
	require.Greater(t, lower.Cmp(upper), 0, "the toe tool leaves a strict whole-side gap")
	section.Start.U, section.End.U, section.End.V = 3.25, 3.25, 3.25
	_, _, outcome = revolveloftfaces.VerticalLimits(trim, section, big.NewRat(13, 4))
	require.Equal(t, revolveloftfaces.VerticalRefusal, outcome,
		"a touching radial boundary does not prove a miss")
	section.Start.U, section.End.U = 8, 8
	_, _, outcome = revolveloftfaces.VerticalLimits(trim, section, big.NewRat(8, 1))
	require.Equal(t, revolveloftfaces.VerticalRefusal, outcome)
}

func TestRevolveLoftFacesConstantXBand(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(x *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{x, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       loftFaceRatVec{zero, zero, one},
			G0:       one, GU: zero, GV: zero,
		}
	}
	box := [4]*big.Rat{zero, one, zero, one}
	maps := []loftFaceMap{makeMap(big.NewRat(5, 1)), makeMap(big.NewRat(6, 1)), makeMap(big.NewRat(7, 1))}
	for _, tc := range []struct {
		x    *big.Rat
		band int
	}{
		{big.NewRat(11, 2), 0},
		{big.NewRat(13, 2), 1},
	} {
		band, time, ok := loftface.ConstantXBand(maps, box, tc.x)
		require.True(t, ok)
		require.Equal(t, tc.band, band)
		require.Equal(t, "1/2", time.RatString())
	}
	_, _, ok := loftface.ConstantXBand(maps, box, big.NewRat(6, 1))
	require.False(t, ok, "a cap coincidence has no unique interior band time")
	maps[2] = makeMap(big.NewRat(11, 2))
	_, _, ok = loftface.ConstantXBand(maps, box, big.NewRat(21, 4))
	require.False(t, ok, "cap X must strictly increase")
	maps[2] = makeMap(big.NewRat(7, 1))
	maps[1].QU[0] = one
	_, _, ok = loftface.ConstantXBand(maps, box, big.NewRat(11, 2))
	require.False(t, ok, "cap X must be source independent")
}

func TestRevolveLoftFacesVerticalClipsLiveSourceSpan(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(x *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{x, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       loftFaceRatVec{zero, zero, one},
			G0:       one, GU: zero, GV: zero,
		}
	}
	span := freeform.BezierSpan{{U: one, V: zero}, {U: big.NewRat(3, 1), V: zero}}
	trim := &loftFacesTrimRecord{Maps: []loftFaceMap{makeMap(big.NewRat(5, 1)), makeMap(big.NewRat(6, 1))}}
	cell, ok := revolvecontact.VerticalBezier(trim, 3, 0, 0,
		big.NewRat(1, 2), big.NewRat(3, 2), big.NewRat(5, 2), provenance.Ref{}, span)
	require.True(t, ok)
	require.Zero(t, cell.Lower.Side)
	require.Equal(t, -1, cell.Lower.Before)
	require.Equal(t, 1, cell.Lower.After)
	require.Zero(t, cell.Upper.Side)
	require.Equal(t, -1, cell.Upper.Before)
	require.Equal(t, 1, cell.Upper.After)
	require.Less(t, cell.Lower.Root.Hi.Cmp(cell.Upper.Root.Lo), 0)
	require.Len(t, cell.Retained, 1)
	require.Equal(t, "1/4", cell.Lower.Root.Hi.RatString())
	rootPoint := loftface.BezierPoint(span, cell.Retained[0].Source.Lo)
	require.Equal(t, "3/2", rootPoint[0].RatString())
	require.Equal(t, 0, cell.Retained[0].Source.Lo.Cmp(cell.Lower.Root.Hi))
	require.Equal(t, 0, cell.Retained[0].Source.Hi.Cmp(cell.Upper.Root.Lo))
	require.True(t, cell.Retained[0].LoOpen,
		"an exact radial clip root is not inside the strict retained interval")
	require.False(t, cell.Retained[0].HiOpen)
	require.Equal(t, "11/2", cell.Retained[0].Box[0].Lo.RatString())
	require.Equal(t, "11/2", cell.Retained[0].Box[0].Hi.RatString())
	for _, limit := range [...]loftface.VerticalLimit{cell.Lower, cell.Upper} {
		require.Less(t, limit.Root.Lo.Cmp(limit.Root.Hi), 0)
		require.Equal(t, "11/2", limit.Box[0].Lo.RatString())
	}
	_, ok = loftface.VerticalClassify([]*big.Rat{zero, one})
	require.False(t, ok)
}

func TestRevolveLoftFacesVerticalAffineOrientation(t *testing.T) {
	zero, one := lfZero(), lfOne()
	makeMap := func(scale *big.Rat) loftFaceMap {
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{zero, zero, zero},
			QU:       loftFaceRatVec{zero, scale, zero},
			QV:       loftFaceRatVec{zero, zero, scale},
			G0:       one, GU: zero, GV: zero,
		}
	}
	first, last := makeMap(big.NewRat(1, 2)), makeMap(one)
	affine, ok := loftface.VerticalAffineMap(first, last, big.NewRat(1, 2))
	require.True(t, ok)
	require.Equal(t, "3/4", affine.Column[0][0].RatString())
	require.Equal(t, "3/4", affine.Column[1][1].RatString())
	require.Equal(t, "9/16", affine.Det.RatString())
	last.QV[2] = big.NewRat(-1, 1)
	_, ok = loftface.VerticalAffineMap(first, last, one)
	require.False(t, ok)
	last.QV[2] = one
	last.GU = one
	_, ok = loftface.VerticalAffineMap(first, last, big.NewRat(1, 2))
	require.False(t, ok)
}

func TestCurvedVerticalSourceConsumesLiveProducer(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	if err != nil || !certified {
		t.Fatalf("live source was not admitted: %v, %t", err, certified)
	}
	manifest, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
	if err != nil || !complete || manifest == nil {
		t.Fatalf("live source did not reach all-side consumer: %v, %t", err, complete)
	}
	if len(manifest.SideOutcome) != len(proof.Section.Spans) {
		t.Fatal("all-side record does not cover the live Revolve section")
	}
	contacts, misses := 0, 0
	for _, entry := range manifest.Vertical {
		if entry.SectionIndex < 0 || entry.SectionIndex >= len(proof.Section.Spans) ||
			entry.Outcome == revolveloftfaces.VerticalContact && entry.Side == nil ||
			entry.Outcome == revolveloftfaces.VerticalMiss && entry.Side != nil {
			t.Fatal("vertical answer lacks its live side record")
		}
		switch entry.Outcome {
		case revolveloftfaces.VerticalContact:
			contacts++
		case revolveloftfaces.VerticalMiss:
			misses++
		default:
			t.Fatal("undecidable vertical side was admitted")
		}
	}
	if contacts == 0 || misses == 0 {
		t.Fatalf("real source did not exercise both vertical answers: contacts=%d misses=%d", contacts, misses)
	}
	t.Run("changed toe cone slope", func(t *testing.T) {
		original := proof.Trim.Toe.Slope
		defer func() { proof.Trim.Toe.Slope = original }()
		proof.Trim.Toe.Slope = lfAdd(original, lfF(0.001))
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("changed heel cone apex", func(t *testing.T) {
		original := proof.Trim.Heel.Apex
		defer func() { proof.Trim.Heel.Apex = original }()
		proof.Trim.Heel.Apex = lfAdd(original, lfF(0.001))
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("changed toe tool", func(t *testing.T) {
		original := proof.Trim.ToeTool
		defer func() { proof.Trim.ToeTool = original }()
		proof.Trim.ToeTool = proof.Trim.HeelTool
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("cap triangle on source face", func(t *testing.T) {
		mutateCurvedCapFaceOwner(t, tooth, proof, true)
	})
	t.Run("source triangle on cap face", func(t *testing.T) {
		mutateCurvedCapFaceOwner(t, tooth, proof, false)
	})
	t.Run("cap face index outside live faces", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		payload.FaceOf = slices.Clone(payload.FaceOf)
		changed := false
		for i, group := range payload.Src {
			if group == proof.Trim.ToeCapGroup {
				payload.FaceOf[i] = len(tooth.Faces()) + 1
				changed = true
			}
		}
		if !changed {
			t.Fatal("the real producer lacks a toe cap triangle")
		}
		tooth.n.Payload = payload
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("extra blank face", func(t *testing.T) {
		shell := blank.Shells()[0]
		original := shell.n.Faces
		defer func() { shell.n.Faces = original }()
		shell.n.Faces = append(slices.Clone(original), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("an unclassified live blank face was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed root section", func(t *testing.T) {
		original := proof.Section.RootStart
		defer func() { proof.Section.RootStart = original }()
		proof.Section.RootStart.U += 0.125
		_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed live root section was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed root cone", func(t *testing.T) {
		original := proof.Section.RootCone
		defer func() { proof.Section.RootCone = original }()
		proof.Section.RootCone.Origin.X += 0.125
		_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed live root cone was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed tooth transform", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		shift, err := r3.Translation(r3.Vec{X: 1})
		if err != nil {
			t.Fatal(err)
		}
		payload.Xform = shift
		tooth.n.Payload = payload
		if revolvecontact.TrimCurrent(bodyNode(tooth), proof.Trim) {
			t.Fatal("changed tooth transform retained a current source record")
		}
		_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed tooth transform was admitted: %v, %t", err, complete)
		}
	})
	t.Run("wrong vertical triangle owner", func(t *testing.T) {
		contactIndex := -1
		var ref provenance.Ref
		var owner *Face
		for _, entry := range manifest.Vertical {
			if entry.Outcome != revolveloftfaces.VerticalContact {
				continue
			}
			for _, cell := range entry.Side.Cells {
				if len(cell.Retained) > 0 {
					contactIndex, ref, owner = entry.SectionIndex, cell.Ref, faceHandle(cell.Face)
					break
				}
			}
			if contactIndex >= 0 {
				break
			}
		}
		if contactIndex < 0 || owner == nil {
			t.Fatal("the live vertical contact has no retained source face")
		}
		rejectChangedCurvedSourceOwner(t, tooth, ref, owner, func() bool {
			side, outcome, err := revolvecontact.VerticalSideClassify(t.Context(), proof, contactIndex)
			return err == nil && side != nil && outcome == revolveloftfaces.VerticalContact
		})
	})
	t.Run("wrong nonvertical triangle owner", func(t *testing.T) {
		var ref provenance.Ref
		var owner *Face
		for _, entry := range manifest.Nonvertical {
			for _, cell := range entry.Cells {
				if cell.Mapped.Segment != proof.Crossings[0].Segment &&
					cell.Mapped.Segment != proof.Crossings[1].Segment {
					ref, owner = cell.Ref, faceHandle(cell.Face)
					break
				}
			}
			if owner != nil {
				break
			}
		}
		if owner == nil {
			t.Fatal("the live nonvertical side has no retained noncrossing source face")
		}
		rejectChangedCurvedSourceOwner(t, tooth, ref, owner, func() bool {
			_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
			return err == nil && complete
		})
	})
}

func assertChangedCurvedTrimRefuses(t *testing.T, tooth *Body,
	proof *revolvecontact.SourceCertificate) {
	t.Helper()
	if revolvecontact.TrimCurrent(bodyNode(tooth), proof.Trim) {
		t.Fatal("changed trim retained a current source record")
	}
	_, certified, err := revolvecontact.CertifySource(t.Context(), proof.Blank, bodyNode(tooth))
	if err != nil || certified {
		t.Fatalf("changed trim reached source admission: %v, %t", err, certified)
	}
	_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
	if err != nil || complete {
		t.Fatalf("changed trim reached all-side consumer: %v, %t", err, complete)
	}
}

func mutateCurvedCapFaceOwner(t *testing.T, tooth *Body,
	proof *revolvecontact.SourceCertificate, capToSource bool) {
	t.Helper()
	original := tooth.n.Payload
	defer func() { tooth.n.Payload = original }()
	payload := original.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	capGroup := proof.Trim.ToeCapGroup
	capTriangle, sourceTriangle := -1, -1
	for i, group := range payload.Src {
		if group == capGroup && capTriangle < 0 {
			capTriangle = i
		}
		if group != proof.Trim.ToeCapGroup && group != proof.Trim.HeelCapGroup && sourceTriangle < 0 {
			sourceTriangle = i
		}
	}
	if capTriangle < 0 || sourceTriangle < 0 ||
		payload.FaceOf[capTriangle] == payload.FaceOf[sourceTriangle] {
		t.Fatal("the real producer lacks separate cap and source faces")
	}
	if capToSource {
		payload.FaceOf[capTriangle] = payload.FaceOf[sourceTriangle]
	} else {
		payload.FaceOf[sourceTriangle] = payload.FaceOf[capTriangle]
	}
	tooth.n.Payload = payload
	assertChangedCurvedTrimRefuses(t, tooth, proof)
}

func rejectChangedCurvedSourceOwner(t *testing.T, tooth *Body, ref provenance.Ref,
	owner *Face, accepts func() bool) {
	t.Helper()
	original := tooth.n.Payload
	defer func() { tooth.n.Payload = original }()
	payload := original.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	other := -1
	for i, face := range tooth.Faces() {
		if face != owner {
			other = i
			break
		}
	}
	if other < 0 {
		t.Fatal("the live source has no other face")
	}
	changed := false
	for i, group := range payload.Src {
		if slices.Contains(payload.Groups[group].Origins, ref) {
			payload.FaceOf[i] = other
			changed = true
			break
		}
	}
	if !changed {
		t.Fatal("the retained source has no triangle")
	}
	tooth.n.Payload = payload
	if accepts() {
		t.Fatal("changed live triangle owner was admitted")
	}
}
