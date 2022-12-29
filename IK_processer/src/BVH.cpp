#include <fstream>
#include <string>
#include "BVH.h"

// コントラクタ
BVH::BVH()
{
	motion = NULL;
	Clear();
}

// コントラクタ
BVH::BVH(const char* bvh_file_name)
{
	motion = NULL;
	Clear();

	Load(bvh_file_name);
}

// デストラクタ
BVH::~BVH()
{
	Clear();
}


// 全情報のクリア
void  BVH::Clear()
{
	unsigned int  i;
	for (i = 0; i < channels.size(); i++)
		delete  channels[i];
	for (i = 0; i < joints.size(); i++)
		delete  joints[i];
	if (motion != NULL)
		delete  motion;

	is_load_success = false;

	file_name = "";
	motion_name = "";

	num_channel = 0;
	channels.clear();
	joints.clear();
	joint_index.clear();

	num_frame = 0;
	interval = 0.0;
	motion = NULL;
}



//
//  BVHファイルのロード
//
void  BVH::Load(const char* bvh_file_name)
{
#define  BUFFER_LENGTH  1024*4

	ifstream  file;
	char      line[BUFFER_LENGTH];
	char* token;
	char      separater[] = " :,\t";
	vector< Joint* >   joint_stack;
	Joint* joint = NULL;
	Joint* new_joint = NULL;
	bool      is_site = false;
	double    x, y, z;
	int       i, j;

	// 初期化
	Clear();

	// ファイルの情報（ファイル名・動作名）の設定
	file_name = bvh_file_name;
	const char* mn_first = bvh_file_name;
	const char* mn_last = bvh_file_name + strlen(bvh_file_name);
	if (strrchr(bvh_file_name, '\\') != NULL)
		mn_first = strrchr(bvh_file_name, '\\') + 1;
	else if (strrchr(bvh_file_name, '/') != NULL)
		mn_first = strrchr(bvh_file_name, '/') + 1;
	if (strrchr(bvh_file_name, '.') != NULL)
		mn_last = strrchr(bvh_file_name, '.');
	if (mn_last < mn_first)
		mn_last = bvh_file_name + strlen(bvh_file_name);
	motion_name.assign(mn_first, mn_last);

	// ファイルのオープン
	file.open(bvh_file_name, ios::in);
	if (file.is_open() == 0)  return; // ファイルが開けなかったら終了

	// 階層情報の読み込み
	int lineNum = 0;
	while (!file.eof())
	{
		// ファイルの最後まできてしまったら異常終了
		if (file.eof())  goto bvh_error;

		// １行読み込み、先頭の単語を取得
		file.getline(line, BUFFER_LENGTH);
		token = strtok(line, separater);
		lineNum++;

		// 空行の場合は次の行へ
		if (token == NULL)  continue;

		// 関節ブロックの開始
		if (strcmp(token, "{") == 0)
		{
			// 現在の関節をスタックに積む
			joint_stack.push_back(joint);
			joint = new_joint;
			continue;
		}
		// 関節ブロックの終了
		if (strcmp(token, "}") == 0)
		{
			// 現在の関節をスタックから取り出す
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		// 関節情報の開始
		if ((strcmp(token, "ROOT") == 0) ||
			(strcmp(token, "JOINT") == 0))
		{
			// 関節データの作成
			new_joint = new Joint();
			new_joint->index = joints.size();
			new_joint->parent = joint;
			new_joint->has_site = false;
			new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
			new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
			joints.push_back(new_joint);
			if (joint)
				joint->children.push_back(new_joint);

			// 関節名の読み込み
			token = strtok(NULL, "");
			while (*token == ' ')  token++;
			new_joint->name = token;

			// インデックスへ追加
			joint_index[new_joint->name] = new_joint;
			continue;
		}

		// 末端情報の開始
		if ((strcmp(token, "End") == 0))
		{
			new_joint = joint;
			is_site = true;
			continue;
		}

		// 関節のオフセット or 末端位置の情報
		if (strcmp(token, "OFFSET") == 0)
		{
			// 座標値を読み込み
			token = strtok(NULL, separater);
			x = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			y = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			z = token ? atof(token) : 0.0;

			// 関節のオフセットに座標値を設定
			if (is_site)
			{
				joint->has_site = true;
				joint->site[0] = x;
				joint->site[1] = y;
				joint->site[2] = z;
			}
			else
				// 末端位置に座標値を設定
			{
				joint->offset[0] = x;
				joint->offset[1] = y;
				joint->offset[2] = z;
			}
			continue;
		}

		// 関節のチャンネル情報
		if (strcmp(token, "CHANNELS") == 0)
		{
			// チャンネル数を読み込み
			token = strtok(NULL, separater);
			joint->channels.resize(token ? atoi(token) : 0);

			// チャンネル情報を読み込み
			for (i = 0; i < joint->channels.size(); i++)
			{
				// チャンネルの作成
				Channel* channel = new Channel();
				channel->joint = joint;
				channel->index = channels.size();
				channels.push_back(channel);
				joint->channels[i] = channel;

				// チャンネルの種類の判定
				token = strtok(NULL, separater);
				if (strcmp(token, "Xrotation") == 0)
					channel->type = X_ROTATION;
				else if (strcmp(token, "Yrotation") == 0)
					channel->type = Y_ROTATION;
				else if (strcmp(token, "Zrotation") == 0)
					channel->type = Z_ROTATION;
				else if (strcmp(token, "Xposition") == 0)
					channel->type = X_POSITION;
				else if (strcmp(token, "Yposition") == 0)
					channel->type = Y_POSITION;
				else if (strcmp(token, "Zposition") == 0)
					channel->type = Z_POSITION;
			}
		}

		// Motionデータのセクションへ移る
		if (strcmp(token, "MOTION") == 0) {
			LineNumOfMotion = lineNum;
			break;
		}
		
	}


	// モーション情報の読み込み
	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, separater);
	if (strcmp(token, "Frames") != 0)  goto bvh_error;
	token = strtok(NULL, separater);
	if (token == NULL)  goto bvh_error;
	num_frame = atoi(token);

	file.getline(line, BUFFER_LENGTH);
	token = strtok(line, ":");
	if (strcmp(token, "Frame Time") != 0)  goto bvh_error;
	token = strtok(NULL, separater);
	if (token == NULL)  goto bvh_error;
	interval = atof(token);

	num_channel = channels.size();
	motion = new double[num_frame * num_channel];

	// モーションデータの読み込み
	for (i = 0; i < num_frame; i++)
	{
		file.getline(line, BUFFER_LENGTH);
		token = strtok(line, separater);
		for (j = 0; j < num_channel; j++)
		{
			if (token == NULL)
				goto bvh_error;
			motion[i * num_channel + j] = atof(token);
			token = strtok(NULL, separater);
		}
	}

	// ファイルのクローズ
	file.close();

	// ロードの成功
	is_load_success = true;

	return;

bvh_error:
	file.close();
}

void  BVH::Write(string bvh_file_name)
{
	ifstream inputFile;
	string content = "";
	char      line[BUFFER_LENGTH];
	char* token;
	char      separater[] = " :,\t";
	inputFile.open(bvh_file_name);
	
	while (!inputFile.eof())
	{
		// １行読み込み、先頭の単語を取得
		inputFile.getline(line, BUFFER_LENGTH);
		content += line;
		content += "\n";
		token = strtok(line, separater);
		if (strcmp(token, "MOTION") == 0) {
			break;
		}
	}
	inputFile.getline(line, BUFFER_LENGTH); content += line; content += "\n";
	inputFile.getline(line, BUFFER_LENGTH); content += line; content += "\n";
	inputFile.close();

	ofstream outputFile;
	outputFile.open(bvh_file_name);
	outputFile << content;

	outputFile << setiosflags(ios::fixed) << setprecision(6);
	if (outputFile.is_open() == 0)  return; // ファイルが開けなかったら終了
	for (int i = 0; i < num_frame; i++) {
		for(int j=0;j< num_channel;j++)
			outputFile <<motion[i* num_channel + j]<<" ";
		outputFile << endl;
	}
	outputFile.close();
}

// End of BVH.cpp