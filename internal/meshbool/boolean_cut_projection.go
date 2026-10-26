package meshbool

import "math/big"

// cutProjectionScratch owns temporary 2D coordinates for consecutive facet
// subdivisions. CutRegion carries only independently owned 3D points, so a
// completed subdivision keeps no reference to these rationals. One worker owns
// the scratch and resets it only after the preceding CutTriangle returns.
type cutProjectionScratch struct {
	rats []*big.Rat
	used int
}

func (s *cutProjectionScratch) rat() *big.Rat {
	if s.used == len(s.rats) {
		s.rats = append(s.rats, new(big.Rat))
	}
	r := s.rats[s.used]
	s.used++
	return r
}
