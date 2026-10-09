package linkagebound

// CellSubdivision schedules joint-box cells in depth and cell order. The
// caller owns each cell's geometry, classification and centre evaluation.
type CellSubdivision[C any] struct {
	Budget           int
	Evaluated        func() int
	Evaluate         func(C) error
	Split            func(C, int) (C, C, error)
	Depth            func(C) int
	VerdictAxis      func(C) int
	NextReadingSplit func([]C) (int, int)

	Leaves    []C
	Exhausted bool
	Unsplit   int
}

// Run evaluates the whole box, then alternates verdict and reading splits.
// Each split consumes two centre evaluations from the same budget.
func (s *CellSubdivision[C]) Run(root C) error {
	if err := s.Evaluate(root); err != nil {
		return err
	}
	s.Leaves = []C{root}
	for {
		if err := s.splitForVerdict(); err != nil {
			return err
		}
		if s.Exhausted {
			return nil
		}
		n, axis := s.NextReadingSplit(s.Leaves)
		if n < 0 {
			return nil
		}
		if s.Evaluated()+2 > s.Budget {
			s.Exhausted = true
			s.Unsplit++
			return nil
		}
		lower, upper, err := s.Split(s.Leaves[n], axis)
		if err != nil {
			return err
		}
		s.Leaves = replaceCell(s.Leaves, n, lower, upper)
	}
}

// splitForVerdict replaces every splittable cell at the shallowest depth,
// from left to right. A spent budget keeps the remaining cells as leaves.
func (s *CellSubdivision[C]) splitForVerdict() error {
	for {
		level := -1
		for _, c := range s.Leaves {
			if s.VerdictAxis(c) >= 0 && (level < 0 || s.Depth(c) < level) {
				level = s.Depth(c)
			}
		}
		if level < 0 {
			return nil
		}
		next := make([]C, 0, len(s.Leaves))
		for _, c := range s.Leaves {
			axis := s.VerdictAxis(c)
			if s.Depth(c) != level || axis < 0 {
				next = append(next, c)
				continue
			}
			if s.Exhausted || s.Evaluated()+2 > s.Budget {
				s.Exhausted = true
				s.Unsplit++
				next = append(next, c)
				continue
			}
			lower, upper, err := s.Split(c, axis)
			if err != nil {
				return err
			}
			next = append(next, lower, upper)
		}
		s.Leaves = next
		if s.Exhausted {
			return nil
		}
	}
}

func replaceCell[C any](cells []C, at int, lower, upper C) []C {
	next := make([]C, 0, len(cells)+1)
	next = append(next, cells[:at]...)
	next = append(next, lower, upper)
	return append(next, cells[at+1:]...)
}
