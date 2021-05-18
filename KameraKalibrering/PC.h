/*
 * PC.h
 *
 *  Created on: 4. maj 2021
 *      Author: jeppe
 */

#ifndef PC_H_
#define PC_H_

#include <iostream>
/*#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
*/
class PC {
public:
	PC();
	void writePCD();
	void readPCD();
	void addPoint();
	void showPC();
	void deletePoint();
	virtual ~PC();
private:
};

#endif /* PC_H_ */
