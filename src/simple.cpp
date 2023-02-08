#include <nori/integrator.h>
#include <nori/scene.h>
NORI_NAMESPACE_BEGIN

    class SimpleIntegrator : public Integrator {
    public:
        SimpleIntegrator(const PropertyList &props) {
            m_energy = props.getColor("energy", Color3f(1.0f));
            m_position = props.getPoint("position", Point3f(0.0f));
        }

        /// Compute the radiance value for a given ray. Just return green here
        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            Vector3f lightDir = m_position - its.p;
            auto lightDist = lightDir.norm();
            lightDir = lightDir / lightDist;
            Ray3f shadowRay(its.p, lightDir, Epsilon, lightDist - Epsilon);
            Intersection shadowIts;
            if (scene->rayIntersect(shadowRay, shadowIts))
                return Color3f(0.0f);
            auto cosTheta = std::max(0.0f, its.shFrame.n.dot(lightDir));
            auto scale = 0.25f / (lightDist * lightDist) / M_PI / M_PI;
            return m_energy * cosTheta * scale;
        }

        /// Return a human-readable description for debugging purposes
        std::string toString() const {
            return "SimpleIntegrator[]";
        }

    private:
        Color3f m_energy;
        Point3f m_position;
    };

    NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END