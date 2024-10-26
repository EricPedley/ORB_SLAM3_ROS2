#include <pointmatcher/PointMatcher.h>
// ProjectOrthoDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::ProjectOrthoDataPointsFilter::ProjectOrthoDataPointsFilter(const Parameters& params):
	DataPointsFilter("ProjectOrthoDataPointsFilter", ProjectOrthoDataPointsFilter::availableParameters(), params),
	zPlane(Parametrizable::get<T>("zPlane"))
{
    LOG_INFO_STREAM("Using ProjectOrthoDataPointsFilter with zPlane=" << zPlane);
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ProjectOrthoDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::ProjectOrthoDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
    const int nbPointsIn = cloud.features.cols();
    typedef PointMatcher<T> PM;
    typedef typename PM::DataPoints DataPoints;
    typedef typename DataPoints::Label Label;
    typedef typename DataPoints::Labels Labels;

    Labels featLabels;
    featLabels.push_back(Label("x", 1));
    featLabels.push_back(Label("y", 1));
    featLabels.push_back(Label("pad", 1));

    Labels descLabels;

    DataPoints cloud_filtered(featLabels, descLabels, nbPointsIn);
    // cloud_filtered.getFeatureViewByName("pad")

    int j = 0;
    for (int i = 0; i < nbPointsIn; i++)
    {
        cloud_filtered.features(0, i) = cloud.features.col(i).array()[0];
        cloud_filtered.features(1, i) = cloud.features.col(i).array()[1];
        cloud_filtered.features(2, i) = 1;
        j++;
    }



    // cloud = cloud_filtered;
    PointMatcher<T>::swapDataPoints(cloud, cloud_filtered);

    // for (int i = 0; i < nbPointsIn; i++)
    // {
    //     std::cout << "Cloud " << cloud.features.col(i).array() << std::endl;
    //     std::cout << "Cloud_filtered " << cloud_filtered.features.col(i).array() << std::endl;
    // }

    // cloud.conservativeResize(j);
}

template struct DataPointsFiltersImpl<float>::ProjectOrthoDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ProjectOrthoDataPointsFilter;
