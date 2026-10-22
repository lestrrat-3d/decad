package sectionrecord

// FitSplinePieces counts the cubic pieces the fit-spline segments of loops
// convert to: one between each pair of fit points. It is the quantity a
// profile extrude's free-form work allowance scales with
// (agent-docs/spline-design.md §5.2). Every other segment kind counts none, so
// its work keeps the default ceiling.
func FitSplinePieces(loops ...LoopRecord) uint64 {
	var n uint64
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			if fit, ok := seg.(FitSplineSeg); ok {
				n += uint64(max(len(fit.Fit)-1, 1))
			}
		}
	}
	return n
}
