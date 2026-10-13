package spiralsector

import (
	"context"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ScopeWidenedContacts checks every exact held contact whose tooth
// facet uses the source-specific Gate allowance. A near-face role alone
// cannot cover a deeper contact hidden inside the Union result.
func ScopeWidenedContacts(ctx context.Context, blankHeld, toothHeld *meshbool.BoolMesh,
	memo *meshbool.ContactMemo, allowed map[[2]int]bool, ordinaryGate float64,
	expected ...int) (bool, error) {
	want := 24
	if len(expected) == 1 {
		want = expected[0]
	}
	maxToothTris := 1 << 17
	if want > 24 {
		maxToothTris = 1 << 21
	}
	if ctx == nil || blankHeld == nil || toothHeld == nil || memo == nil ||
		len(allowed) != want || ordinaryGate <= 0 ||
		len(toothHeld.FacetBound) != len(toothHeld.Tris) ||
		len(blankHeld.Src) != len(blankHeld.Tris) ||
		len(toothHeld.Src) != len(toothHeld.Tris) ||
		len(blankHeld.Tris) > 1<<15 || len(toothHeld.Tris) > maxToothTris {
		return false, nil
	}
	if want < 24 || want%24 != 0 || want > 24*31 {
		return false, nil
	}
	for _, bound := range toothHeld.FacetBound {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return false, nil
		}
	}
	// Sorted exact float AABB endpoints enumerate every overlapping X span.
	// The later full box test still owns Y/Z and equality at the boundary.
	blankX := make([]int, len(blankHeld.Tris))
	for i := range blankX {
		blankX[i] = i
	}
	sort.Slice(blankX, func(i, j int) bool {
		a, b := blankHeld.Boxes[blankX[i]][0].X, blankHeld.Boxes[blankX[j]][0].X
		if a != b {
			return a < b
		}
		return blankX[i] < blankX[j]
	})
	// A composite has one authenticated copy of the same 24-role tooth per
	// sector. Bound cheap X scans per copy, while limiting the full 3D box
	// candidates sent to the exact contact memo across the whole operand.
	xScans, contacts := 0, 0
	maxXScans := (want / 24) * (1 << 24)
	for j := range toothHeld.Tris {
		if toothHeld.FacetBound[j] <= ordinaryGate {
			continue
		}
		toothBox := toothHeld.Boxes[j]
		limit := sort.Search(len(blankX), func(k int) bool {
			return blankHeld.Boxes[blankX[k]][0].X > toothBox[1].X
		})
		for _, i := range blankX[:limit] {
			if blankHeld.Boxes[i][1].X < toothBox[0].X {
				continue
			}
			xScans++
			if xScans > maxXScans {
				return false, nil
			}
			if xScans%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			if !meshbool.BoxesWithin(blankHeld.Boxes[i], toothBox, 0) {
				continue
			}
			contacts++
			if contacts > 1<<24 {
				return false, nil
			}
			contact, err := memo.Classify(i, j)
			if err != nil {
				return false, err
			}
			if contact.Kind != meshbool.ContactNone &&
				!allowed[[2]int{blankHeld.Src[i], toothHeld.Src[j]}] {
				return false, nil
			}
		}
	}
	return ctx.Err() == nil, ctx.Err()
}

// ScopeOutputSwitches checks every stitched cross-operand edge, even
// when its input contact had a facet bound below the ordinary pair gate.
func ScopeOutputSwitches(ctx context.Context, held *meshbool.StitchedMesh,
	blankFaces int, allowed map[[2]int]bool, expected ...int) (bool, error) {
	want := 24
	if len(expected) == 1 {
		want = expected[0]
	}
	maxOutputTris := 1 << 17
	if want > 24 {
		maxOutputTris = 1 << 21
	}
	if ctx == nil || held == nil || blankFaces <= 0 || len(allowed) != want ||
		len(held.Tris) > maxOutputTris || len(held.Src) != len(held.Tris) {
		return false, nil
	}
	type edge struct{ a, b int }
	owners := make(map[edge][2]int, len(held.Tris)*3/2)
	counts := make(map[edge]uint8, len(held.Tris)*3/2)
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			if a < 0 || b < 0 || a >= len(held.Verts) || b >= len(held.Verts) || a == b {
				return false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := edge{a, b}
			if counts[key] >= 2 {
				return false, nil
			}
			pair := owners[key]
			pair[counts[key]] = held.Src[i]
			owners[key] = pair
			counts[key]++
		}
	}
	seen := 0
	for edge, pair := range owners {
		_ = edge
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		seen++
		if counts[edge] != 2 {
			return false, nil
		}
		a, b := pair[0], pair[1]
		if a >= blankFaces && b < blankFaces {
			a, b = b, a
		}
		if a < blankFaces && b >= blankFaces && !allowed[[2]int{a, b}] {
			return false, nil
		}
	}
	return ctx.Err() == nil, ctx.Err()
}
