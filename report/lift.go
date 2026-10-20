package report

// Lift maps one instantiation's body, face, edge and vertex references to
// another's. A nil function is a bug; each maps the nil reference to nil.
type Lift[B1, F1, E1, V1, B2, F2, E2, V2 any] struct {
	Body   func(B1) B2
	Face   func(F1) F2
	Edge   func(E1) E2
	Vertex func(V1) V2
}

// Feature maps a feature.
func (l Lift[B1, F1, E1, V1, B2, F2, E2, V2]) Feature(f ContactFeatureOf[F1, E1, V1]) ContactFeatureOf[F2, E2, V2] {
	return ContactFeatureOf[F2, E2, V2]{Face: l.Face(f.Face), Edge: l.Edge(f.Edge), Vertex: l.Vertex(f.Vertex)}
}

// Point maps a point.
func (l Lift[B1, F1, E1, V1, B2, F2, E2, V2]) Point(p ContactPointOf[F1, E1, V1]) ContactPointOf[F2, E2, V2] {
	return ContactPointOf[F2, E2, V2]{OnA: p.OnA, OnB: p.OnB, Normal: p.Normal, NormalAngle: p.NormalAngle,
		Separation: p.Separation, FaceA: l.Face(p.FaceA), FaceB: l.Face(p.FaceB),
		FeatureA: l.Feature(p.FeatureA), FeatureB: l.Feature(p.FeatureB)}
}

// Manifold maps a manifold; nil stays nil and empty stays empty.
func (l Lift[B1, F1, E1, V1, B2, F2, E2, V2]) Manifold(m *ContactManifoldOf[F1, E1, V1]) *ContactManifoldOf[F2, E2, V2] {
	if m == nil {
		return nil
	}
	out := &ContactManifoldOf[F2, E2, V2]{}
	if m.Points != nil {
		out.Points = make([]ContactPointOf[F2, E2, V2], len(m.Points))
		for i := range m.Points {
			out.Points[i] = l.Point(m.Points[i])
		}
	}
	return out
}

// Report maps a report.
func (l Lift[B1, F1, E1, V1, B2, F2, E2, V2]) Report(r *ContactReportOf[B1, F1, E1, V1]) *ContactReportOf[B2, F2, E2, V2] {
	return &ContactReportOf[B2, F2, E2, V2]{A: l.Body(r.A), B: l.Body(r.B), PoseA: r.PoseA, PoseB: r.PoseB,
		Request: r.Request, Relation: r.Relation, Gap: copyMeasurement(r.Gap), Overlap: copyMeasurement(r.Overlap),
		Manifold: l.Manifold(r.Manifold), Reason: r.Reason}
}

// copyMeasurement returns a fresh copy of m, or nil for nil, so a lifted report
// shares no reading with the report it came from.
func copyMeasurement(m *Measurement) *Measurement {
	if m == nil {
		return nil
	}
	c := *m
	return &c
}
