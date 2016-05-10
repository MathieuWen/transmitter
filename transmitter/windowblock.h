#include <stdint.h>
class WindowBlock
{
public:
	int index;
	int window_index; // Window���s��, �]�O�Ҧb���ɮ׵����m (FILE Position)
	unsigned char *data;  // Window�����
	double overhead; // overhead
	int window_size;  // ��ڪ�Window���j�p (Byte), default: 2M
	int block_size;  // �������Window����Message Block�j�p (Byte), default: 1K
	int block_num; // �������Window�������X��Message Block, default: 2000
	int *layer_index; // �U��Layer���_�l�����m
	int *layer_size; // Layeri����Message Block�j�p n
	double *layer_weight; // �U��Layer�Q�D�襤�����v
	int layer_num;  // Window�����X��Layer, �b���u��@2�h, default: 2
	int layer_last;
	int m; // m=0, �N���̫᪺Window Block, m=1�N���٦��䥦��Window Block
};