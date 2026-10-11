package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// chordPartialChamferBand uses one side/cap column at each end of the two
// selected straight walks. The recorded patch planes contain every cell.
func chordPartialChamferBand(ctx context.Context, b brepLoopBand, f brepFace,
	cbp capBlendPayload, chord float64) (brepBandChord, error) {
	work := freeform.NewFreeformWork()
	side, err := oneLoopCornerLoop(proofbound.NewWorkBudget(ctx), b.orig, work)
	if err != nil {
		return brepBandChord{}, err
	}
	partial, err := partialband.PartialFilletLayout(side.walks, b.orig.Segments,
		f.regionLoop(b.loop).Segments, b.selected, b.capWalk, b.capArc,
		b.setback.dc, chord, work)
	if err != nil {
		return brepBandChord{}, err
	}
	for _, walk := range partial.Walks {
		if walk.Sag != 0 || len(walk.Cols) != 2 {
			return brepBandChord{}, fmt.Errorf(`%w: a selected chamfer walk is not straight`, ErrUnsupported)
		}
	}
	sideZ, sideDelta := b.sideLevel(f)
	return brepBandChord{band: b, face: f, cbp: cbp, partial: partial,
		start: b.matSign(f) > 0, sideZ: sideZ, sideDelta: sideDelta}, nil
}

func (bc *brepBandChord) emitPartialChamfer(m *Mesh, faceOfRole func(string) (*Face, error),
	bump func(*Face, float64)) error {
	for p := range bc.partial.PatchSag {
		face, err := faceOfRole(bc.band.patchRole(p))
		if err != nil {
			return err
		}
		first := len(m.triangles)
		for _, cell := range bc.partial.Cells {
			if cell.Patch != p {
				continue
			}
			a, b := bc.sideV[cell.C0], bc.sideV[cell.C1]
			A, B := bc.capV[cell.C0], bc.capV[cell.C1]
			if bc.start {
				m.addTriangle([3]int{A, B, b}, face)
				m.addTriangle([3]int{A, b, a}, face)
			} else {
				m.addTriangle([3]int{a, b, B}, face)
				m.addTriangle([3]int{a, B, A}, face)
			}
		}
		// The exact rational coplanarity gate and zero contour/level
		// displacement make the patch deviation zero. Stored vertices still
		// contribute to the body's motion and area allowances.
		bump(face, 0)
		m.areaSlack = proofbound.AbsSumUpper(m.areaSlack,
			tessellation.FilletMeshAreaDeficit(m.vertices, m.triangles[first:], face.area, face.areaBound))
	}
	return nil
}
