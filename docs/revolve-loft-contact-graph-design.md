# Revolve and LoftFaces contact graph

This private graph names source events shared by a finite Revolve boundary and
an authenticated, cone-trimmed `LoftFaces` body. The producer is the current
`certifyRevolveLoftFacesSource` certificate and its complete face-pair census.
The graph does not establish a closed joined boundary or admit `Union`.

## Source ownership

The graph has one entry for every ordered pair of live Revolve and trimmed
`LoftFaces` faces. It also has one entry for every source cell in the census
manifest, including strict-side cells on pair misses and vertical cells with
no retained wall. A retained cell belongs to exactly one pair. Its identity
includes the live face, `FeatureRef`, source segment and span, LoftFaces band,
and exact source and band-time intervals. Miss cells own no seam station ID.

An occupied heel-cap arc has one pair entry and two distinct endpoints. Each
endpoint must have the unique source-side cell proved by
`revolveLoftFacesHeelArcSideTiesOf`. A whole radial Sketch arc contact has one
complete station run. All cap endpoints and stations receive IDs from one
counter, so unrelated event kinds cannot share an ID. No ID is inferred from
nearby held coordinates.

## Held anchors and freshness

The checked open-patch constructor rederives the graph, selects one whole-arc
run by exact source interval and live face, and binds each station ID to the
actual seam index in both open patches. It checks the shared held point and
each side's vertex bound.

The cap-ring blank-shell constructor selects a graph run and its occupied cap
arcs. After building, it binds every station ID to the actual shell seam index
and compares that point with both checked open patches. Each cap endpoint ID
must name exactly one ring vertex and retain its unique tied side-cell ID.
The ring's held point and blank-side bound are checked. Duplicate or missing
shell, patch, ring, or side-cell anchors refuse.

A stored graph is checked against a newly certified source and census before
later use. The comparison includes every pair verdict, cell owner and exact
interval, complete station run, and occupied cap arc. A changed source
station, face, reference, interval, root, or event ID refuses. Graph records
and the open patches remain construction pieces; the retained LoftFaces faces,
joined shell, boundary displacement, and occupied-volume proofs are separate.
