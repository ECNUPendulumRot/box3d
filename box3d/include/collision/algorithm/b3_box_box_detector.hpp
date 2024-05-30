
#ifndef B3_BOX_BOX_DETECTOR_H
#define B3_BOX_BOX_DETECTOR_H

/// this is from bullet3

class b3Fixture;
class b3ManifoldResult;

class b3BoxBoxDetector
{
    const b3Fixture* m_fixtureA;
    const b3Fixture* m_fixtureB;

public:
	b3BoxBoxDetector(const b3Fixture* fixtureA, const b3Fixture* fixtureB);

	void get_closest_points(b3ManifoldResult& output);
};

#endif
