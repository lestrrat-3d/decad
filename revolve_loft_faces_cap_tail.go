package decad

import (
	"context"
	"math/big"
)

type revolveLoftFacesHeelTailBand struct {
	band   int
	lo, hi *big.Rat
	margin [2]*big.Rat
}

// The entire live LoftFaces source lies radially beyond the heel cone from
// the terminal witness until the cone apex or the final source cap. Every
// rational margin is strict; this source-only record selects no held facets.
type revolveLoftFacesHeelTail struct {
	terminal *revolveLoftFacesHeelTerminal
	bands    []revolveLoftFacesHeelTailBand
	endX     *big.Rat
	capFace  *Face
}

func certifyRevolveLoftFacesHeelTail(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (
	*revolveLoftFacesHeelTail, bool, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.trim.source == nil ||
		proof.trim.heel == nil || proof.trim.heel.apex == nil ||
		proof.trim.heel.slope == nil || proof.trim.heel.slope.Sign() <= 0 {
		return nil, false, nil
	}
	terminal, ok, err := certifyRevolveLoftFacesHeelTerminal(ctx, proof)
	if err != nil || !ok || terminal == nil || terminal.endTime == nil ||
		terminal.upper == nil || terminal.capFace != proof.heelFace {
		return nil, false, err
	}
	trim := proof.trim
	source, err := lfSourceBox(trim.source)
	if err != nil {
		return nil, false, err
	}
	corners := lfBoxCorners(source)
	yAxis := loftFaceRatVec{lfZero(), lfOne(), lfZero()}
	if terminal.upper.band < 0 || terminal.upper.band+1 >= len(trim.maps) {
		return nil, false, nil
	}
	startX := lfF(terminal.meridian.end.U)
	if startX.Cmp(trim.heel.apex) >= 0 {
		return nil, false, nil
	}
	result := &revolveLoftFacesHeelTail{terminal: terminal, capFace: proof.heelFace}
	previousX := startX
	for band := terminal.upper.band; band+1 < len(trim.maps); band++ {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if !revolveLoftFacesHeelRegionAffine(trim, band) {
			return nil, false, nil
		}
		x0, first := revolveLoftFacesConstantX(trim.maps[band])
		x1, last := revolveLoftFacesConstantX(trim.maps[band+1])
		if !first || !last || x0.Cmp(x1) >= 0 {
			return nil, false, nil
		}
		lo := lfZero()
		if band == terminal.upper.band {
			lo = terminal.endTime
		}
		hi := lfOne()
		if x1.Cmp(trim.heel.apex) > 0 {
			hi = lfDiv(lfSub(trim.heel.apex, x0), lfSub(x1, x0))
		}
		if lo.Sign() < 0 || lo.Cmp(hi) >= 0 || hi.Cmp(lfOne()) > 0 ||
			lfAdd(lfMul(lfSub(lfOne(), lo), x0), lfMul(lo, x1)).Cmp(previousX) != 0 {
			return nil, false, nil
		}
		firstY := lfScalarRange(trim.maps[band], yAxis, corners)
		lastY := lfScalarRange(trim.maps[band+1], yAxis, corners)
		entry := revolveLoftFacesHeelTailBand{band: band, lo: lo, hi: hi}
		for index, time := range [...](*big.Rat){lo, hi} {
			x := lfAdd(lfMul(lfSub(lfOne(), time), x0), lfMul(time, x1))
			y := mappedArcBlend(firstY, lastY, time).lo
			radius := lfDiv(lfSub(trim.heel.apex, x), trim.heel.slope)
			entry.margin[index] = lfSub(y, radius)
			if radius.Sign() < 0 || entry.margin[index].Sign() <= 0 {
				return nil, false, nil
			}
		}
		previousX = lfAdd(lfMul(lfSub(lfOne(), hi), x0), lfMul(hi, x1))
		result.bands = append(result.bands, entry)
		if previousX.Cmp(trim.heel.apex) == 0 {
			break
		}
	}
	if len(result.bands) == 0 || result.bands[0].lo.Cmp(terminal.endTime) != 0 ||
		previousX.Cmp(trim.heel.apex) < 0 &&
			result.bands[len(result.bands)-1].band+1 != len(trim.maps)-1 {
		return nil, false, nil
	}
	result.endX = previousX
	return result, true, nil
}
