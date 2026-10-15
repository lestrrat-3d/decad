package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

func TestCurvedMappedSourceConsumesLiveProducer(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	if err != nil || !ok {
		t.Fatalf("live source was not admitted: %v, %t", err, ok)
	}
	section := proof.section.spans[proof.section.rootSpan]
	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	if err != nil || !ok {
		t.Fatalf("live source did not reach mapped finite-span consumer: %v, %t", err, ok)
	}
	if len(cells) == 0 {
		t.Fatal("mapped finite-span consumer returned no cells")
	}
	t.Run("exact rows of live mapped span", func(t *testing.T) {
		crossing := proof.crossings[0]
		first, last := trim.Maps[0], trim.Maps[1]
		wantQ := spiralRootPolynomialRef(first, last, crossing.span, proof.rootM, proof.rootB)
		gotQ := loftface.MeridianQRows(first, last, crossing.span, proof.rootM, proof.rootB)
		for i := range gotQ {
			requireSameRows(t, wantQ[i], gotQ[i])
		}
		wantJ := spiralRootXOrderRef(first, last, crossing.span, wantQ)
		gotJ := loftface.MeridianJRows(first, last, crossing.span, gotQ)
		for i := range gotJ {
			requireSameRows(t, wantJ[i], gotJ[i])
		}
		for _, row := range gotQ {
			for _, parameter := range []*big.Rat{
				big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(2, 3), big.NewRat(1, 1),
			} {
				reference := spiralBernValueRef(row, parameter)
				require.Equal(t, reference.Sign(), polynomial.BernsteinSign(row, parameter))
			}
		}
	})
	t.Logf("root section %d mapped %d source cells", proof.section.rootSpan, len(cells))
	radial := 0
	for segment, entity := range trim.Source.source.entities {
		if _, isArc := entity.(*sketch.Arc); !isArc {
			continue
		}
		for band := 0; band+1 < len(trim.Maps); band++ {
			if _, certified := mappedArcRadialTimeBand(t.Context(), trim, segment, band, section); certified {
				radial++
			}
		}
	}
	if radial == 0 {
		t.Fatal("no live mapped arc reached the radial source consumer")
	}
	t.Logf("mapped %d whole-arc bands", radial)
	manifest, complete, manifestErr := revolveLoftFacesNonverticalManifest(t.Context(), proof)
	if manifestErr != nil || !complete || len(manifest) == 0 {
		t.Fatalf("live nonvertical source was not certified: %v, %t", manifestErr, complete)
	}
	var ref provenance.Ref
	var owner *Face
	for _, side := range manifest {
		if len(side.cells) == 0 {
			t.Fatalf("nonvertical section %d has no retained source cells", side.sectionIndex)
		}
		for _, cell := range side.cells {
			if cell.mapped.segment != proof.crossings[0].segment &&
				cell.mapped.segment != proof.crossings[1].segment && owner == nil {
				ref, owner = cell.ref, cell.face
			}
		}
	}
	if owner == nil {
		t.Fatal("the real source has no retained noncrossing wall")
	}
	t.Run("missing noncrossing owner", func(t *testing.T) {
		original := owner.n.Origins
		defer func() { owner.n.Origins = original }()
		owner.n.Origins = slices.DeleteFunc(slices.Clone(original),
			func(got provenance.Ref) bool { return got == ref })
		_, certified, err := revolveLoftFacesNonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("missing live origin was admitted: %v, %t", err, certified)
		}
	})
	t.Run("wrong noncrossing triangle owner", func(t *testing.T) {
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
			t.Fatal("the real source has no second face")
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
			t.Fatal("the retained wall has no source triangle")
		}
		tooth.n.Payload = payload
		_, certified, err := revolveLoftFacesNonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("wrong live face was admitted: %v, %t", err, certified)
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
		if revolveLoftFacesTrimCurrent(tooth, proof.trim) {
			t.Fatal("changed tooth transform retained a current source record")
		}
		_, certified, err := revolveLoftFacesNonverticalManifest(t.Context(), proof)
		if err != nil || certified {
			t.Fatalf("changed tooth transform was admitted: %v, %t", err, certified)
		}
	})
}
