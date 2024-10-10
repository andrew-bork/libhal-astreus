x_pred[0] = q.w + q.x*(b.x - w.x) + q.y*(b.y - w.y) + q.z*(b.z - w.z);
x_pred[1] = q.w*(-b.x + w.x) + q.x + q.y*(-b.z + w.z) + q.z*(b.y - w.y);
x_pred[2] = q.w*(-b.y + w.y) + q.x*(b.z - w.z) + q.y + q.z*(-b.x + w.x);
x_pred[3] = q.w*(-b.z + w.z) + q.x*(-b.y + w.y) + q.y*(b.x - w.x) + q.z;
x_pred[4] = b.x;
x_pred[5] = b.y;
x_pred[6] = b.z;