/**
  ******************************************************************************
             Copyright(c) 2025 Focaltech Systems CO.,Ltd.
                      All Rights Reserved
  ******************************************************************************
  * @file    libAlgorithm.h
  * @author  xukai
  * @version V1.0
  * @date    2021.09.23
  * @brief   Header file of libAlgorithm.
  *
  ******************************************************************************
*/

#ifndef __ALGO_H__
#define __ALGO_H__
#include "algo_drv.h"
typedef HASH_TypeDef SHA_TypeDef;
typedef uint8_t UINT8;
typedef uint32_t UINT32;

/*******************************************************************************
 * Function Name  : LIB_Alg_GetVersion
 * Description    : 算法版本号
 * Input          : None
 * Output         : -version:算法版本号
 * Return         : return 0: success, 1: fail
 ******************************************************************************/
extern unsigned int LIB_Alg_GetVersion(char *version);

/*******************************************************************************
 * Function Name  : LIB_SHA_Enable
 * Description    : 使能SHA模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SHA_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_SHA_Disable
 * Description    : 禁用SHA模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SHA_Disable(void);

/*******************************************************************************
 * Function Name  : LIB_SHA_Init
 * Description    : SHA算法模块初始化
 * Input          : pSha - 哈希算法数据结构
 *                  Mode - 哈希算法编号，取值范围0~4
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SHA_Init(SHA_TypeDef *pSha, UINT8 mode);

/*******************************************************************************
 * Function Name  : LIB_SHA_Update
 * Description    : 发送要进行SHA运算的数据
 * Input          : pSha - 哈希算法数据结构
 *                  pMessageBlock – 指向本次待处理的数据，地址最好4字节对齐
 *                  DataLen – 本次待处理数据长度，长度最好为4的整数倍
 * Output         : None
 * Return         : 0 - 成功
 *                  1 - 失败
 ******************************************************************************/
unsigned int LIB_SHA_Update(SHA_TypeDef *pSha, unsigned char *pMessageBlock, unsigned int DataLen);

/*******************************************************************************
 * Function Name  : LIB_SHA_Final
 * Description    : 结束SHA操作并获取结果
 * Input          : pSha - 哈希算法数据结构
 * Output         : pRes – 哈希运算结果
  * Return         : None
 ******************************************************************************/
unsigned int LIB_SHA_Final(SHA_TypeDef *pSha, unsigned int *pRes);

/*******************************************************************************
 * Function Name  : LIB_RSA_Enable
 * Description    : 使能RSA模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_RSA_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_RSA_Disable
 * Description    : 禁用RSA模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_RSA_Disable(void);

/*******************************************************************************
 * Function Name  : LIB_RSA_GenerateKeyPair
 * Description    : RSA 密钥对生产
 * Input          : mode       RSA算法位数，应填写值为，1024、2048等。
 *                  fixkey     公钥形式
 *                      0 - 公钥不固定，随机产生
 *                      1 - 公钥固定为0x10001
 *                      2 - 公钥外部导入
 *                  crt        RSA密钥格式
 *                      0 - 密钥格式为普通格式
 *                      1 - 密钥格式为CRT格式
 * Output         : pub_key    输出公钥地址
 *                  prv_key    输出私钥地址
 * Return         : 0 - 成功
 *                  1 - 失败
 ******************************************************************************/
unsigned char LIB_RSA_GenerateKeyPair(UINT32 mode, STU_RSA_PUBKEY *pub_key, STU_RSA_PRIVKEY *prv_key, UINT8 fixkey, UINT8 crt);

/*******************************************************************************
* Function Name  : LIB_RSA_PubKey
* Description    : RSA 公钥运算
* Input          : input        输入数据地址
*                  inputLen     输入数据长度
*                  publicKey    RSA公钥地址
*
* Output         : output       输出数据地址
*                  outputLen    输出数据长度

* Return         : 0   - 成功
*                  1   - 失败
******************************************************************************/
int LIB_RSA_PubKey(unsigned int *output, unsigned int *outputLen, unsigned int *input, unsigned int inputLen, STU_RSA_PUBKEY *publicKey);

/*******************************************************************************
* Function Name  : LIB_RSA_PrivKey
* Description    : RSA 私钥运算
* Input          : input        输入数据地址
*                  inputLen     输入数据长度
*                  privateKey   RSA私钥地址
*
* Output         : output       输出数据地址
*                  outputLen    输出数据长度

* Return         : 0   - 成功
*                  1   - 失败
******************************************************************************/
int LIB_RSA_PrivKey(unsigned int *output, unsigned int *outputLen,
                    unsigned int *input, unsigned int inputLen,
                    STU_RSA_PRIVKEY *privateKey);

/*******************************************************************************
* Function Name  : LIB_RSA_PrivKeyCRT
* Description    : RSA 带CRT私钥运算
* Input          : input        输入数据地址
*                  inputLen     输入数据长度
*                  pprivateKey  RSA私钥地址（密钥格式为CRT格式）
*
* Output         : output       输出数据地址
*                  outputLen    输出数据长度

* Return         : 0   - 成功
*                  1   - 失败
******************************************************************************/
int LIB_RSA_PrivKeyCRT(unsigned int *output, unsigned int *outputLen,
                unsigned int *input, unsigned int inputLen,
                STU_RSA_PRIVKEY * pprivateKey);

/*******************************************************************************
 * Function Name  : LIB_ECDSA_Sign
 * Description    : 使用私钥对明文进行签名；
 *
 *                 备注：
 *                 1.此函数内部使用随机数模块，需要打开模块时钟;
 *                 2.此函数内部使用EDAMC模块，需要打开模块时钟;
 *                 3.此函数内部使用CRYPTO模块，需要请打开模块时钟;
 *
 * Input          : mes                     明文
 *                  klen                    明文字节长度
                     hashModeNum            杂凑类型选择
 *                  pstuEccPrams            椭圆曲线参数
 *                  pstuPrivKey             私钥
 * Output         : pSignatureR             输出明文的签名信息R
                    pSignatureS             输出明文的签名信息S
* Return         : 0   - 成功
*                  1   - 失败
 ******************************************************************************/
UINT8 LIB_ECDSA_Sign(unsigned char *mes,unsigned short klen,unsigned char hashModeNum,ECC_STU_BIGINT32 *pstuPrivKey,ECC_STU_BIGINT32 *pSignatureR,ECC_STU_BIGINT32 *pSignatureS,SM2_STU_PRAMS *pstuEccPrams);

/*******************************************************************************
 * Function Name  : LIB_ECDSA_Verify
 * Description    : 使用公钥对明文和签名进行验证；
 *
 *                 备注：
 *                 1.此函数内部使用EDAMC模块，需要打开模块时钟;
 *                 2.此函数内部使用CRYPTO模块，需要打开模块时钟;
 *
 * Input          : mes               明文
 *                  klen              明文字节长度
                    hashModeNum       杂凑类型选择
 *                  pstuEccPrams      椭圆曲线参数
 *                  pstuPubKey        公钥
 *                  pSignatureR       明文的签名信息R
 *                  pSignatureS       明文的签名信息S
 * Output         : None
 * Return         : 0 - 验证通过
 *                  1 - 验证失败
 ******************************************************************************/
UINT8 LIB_ECDSA_Verify(unsigned char *mes,unsigned short klen,unsigned char hashModeNum,SM2_STU_PUBKEY *pstuPubKey,ECC_STU_BIGINT32 *pSignatureR,ECC_STU_BIGINT32 *pSignatureS,SM2_STU_PRAMS *pstuEccPrams);

/*******************************************************************************
 * Function Name  : LIB_SM2_Enable
 * Description    : SM2模块使能
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SM2_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_SM2_Disable
 * Description    : SM2模块禁用
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SM2_Disable(void);

/*******************************************************************************
 * Function Name  : LIB_SM2_GenerateKeyPair
 * Description    : SM2生成的密钥对
 *
 *                 备注：
 *                 1：此函数内部会使用随机数模块，需打开随机数模块时钟并调用InitTrng函数；
 *                 2：函数内部使用EDMAC，需要主动打开EDMAC模块时钟；
 *                 3：内部使用CRYPTO模块，需要主动打开CRYPTO模块时钟；
 *
 * Input          : pstuSM2Prams->uBits              模数P的位数
 *                  pstuSM2Prams->stuCoefficientA    椭圆系数A
 *                  pstuSM2Prams->stuCoefficientA    椭圆系数B
 *                  pstuSM2Prams->stuGx              椭圆基点坐标Gx
 *                  pstuSM2Prams->stuGy              椭圆基点坐标Gy
 *                  pstuSM2Prams->stuPrimeN          椭圆基点坐标G的阶
 *                  pstuPrivKey                      存放生成的私钥
 *                  pstuPbuKey                       存放生成的公钥
 * Output         : None
 * Return         : 0 - 密钥对产生正确
 *                  1 - 密钥对产生失败
 ******************************************************************************/
UINT8 LIB_SM2_GenerateKeyPair(SM2_STU_PRAMS *pstuEccPrams, ECC_STU_BIGINT32 *pstuPrivKey, SM2_STU_PUBKEY *pstuPubKey);

/*******************************************************************************
 * Function Name  : LIB_SM2_Decrypt_V2
 * Description    : 使用私钥对密文解密
 *                  SM2Decrypt_V2：使用私钥对存储方式为C1|C3|C2的密文进行解密；
 *
 *                 备注：
 *                 1.函数内部使用EDMAC，需要打开EDMAC模块时钟；
 *                 2.函数内部使用SHA模块，需要打开SHA模块时钟；
 *                 3.内部使用CRYPTO模块，需要打开CRYPTO模块时钟；
 *                 4.原库：使用私钥对存储方式为C1|C3|C2的密文进行解密
 *
 * Input          : cipher        密文，大端结构
 *                  klen          明文字节长度
 *                  stuPrivKey    私钥
 *                  pstuEccPrams  椭圆曲线参数
 *                  plain：       输出明文，大端结构
 * Output         : None
 * Return         : 0 - 成功
 *                  1 - 失败
 ******************************************************************************/
UINT8 LIB_SM2_Decrypt_V2(unsigned char *cipher, unsigned short klen, ECC_STU_BIGINT32 *stuPrivKey, SM2_STU_PRAMS *pstuEccPrams, unsigned char *plain);

/*******************************************************************************
 * Function Name  : LIB_SM2_Encrypt_V2
 * Description    : 使用公钥对明文加密，产生的密文长度 = 明文长度 + 96字节
 *                  SM2Encrypt_V2：使用公钥对明文加密，成生的密文存储方式为：C1|C3|C2；
 *
 *                 备注：
 *                 1.函数内部使用EDMAC，需要打开EDMAC模块时钟；
 *                 2.函数内部使用SHA模块，需要打开SHA模块时钟；
 *                 3.内部使用CRYPTO模块，需要打开CRYPTO模块时钟；
 *
 * Input          : mes            明文，高字节在后，低字节在前（大端结构），mes[0]为高字节，若最后一个字节不满32位，则在该字节的末尾填0直到满32位;
 *                                 例如mes若为48bit长的123456789abc，则mes[0]为0x12345678，mes[1]为0x9abc0000，
 *                  klen           明文字节长度
 *                  pstuPubKey     公钥
 *                  pstuEccPrams   椭圆曲线参数
 *                  cipher         存放输出的密文，存储方式和明文相同
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SM2_Encrypt_V2(unsigned char *mes, unsigned short klen, SM2_STU_PUBKEY *pstuPubKey, SM2_STU_PRAMS *pstuEccPrams, unsigned char *cipher);

/*******************************************************************************
 * Function Name  : LIB_SM2_Sign
 * Description    : 输出用户的身份信息
 *
 *                 备注：
 *                 1.此函数内部使用SM2模块，需要打开模块时钟;
 *                 2.此函数内部使用EDMAC模块，需要打开模块时钟;
 *
 * Input          : IDA                  签名方的身份信息
 *                  Entla                签名方身份信息的字节长度（备注：原库为签名方身份信息的bit长度）
 *                  pstuPubKey           公钥
 *                  pstuEccPrams         椭圆曲线参数
 *                  za                   输出用户的身份信息
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SM2_Sign(unsigned char *IDA, unsigned short entla, SM2_STU_PUBKEY *pstuPubKey, SM2_STU_PRAMS *pstuEccPrams, unsigned int *za);

/*******************************************************************************
 * Function Name  : LIB_SM2_Signature
 * Description    : 使用私钥对明文进行签名；
 *
 *                 备注：
 *                 1.此函数内部使用随机数模块，需要打开模块时钟;
 *                 2.此函数内部使用EDAMC模块，需要打开模块时钟;
 *                 3.此函数内部使用CRYPTO模块，需要请打开模块时钟;
 *
 * Input          : mes                     明文
 *                  klen                    明文字节长度
 *                  pstuEccPrams            椭圆曲线参数
 *                  pstuPubKey              公钥
 *                  pSignatureR             输出明文的签名信息R
 *                  pSignatureS             输出明文的签名信息S
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SM2_Signature(unsigned char *mes, unsigned short klen, SM2_STU_PRAMS *pstuEccPrams, ECC_STU_BIGINT32 *pstuPrivKey, ECC_STU_BIGINT32 *pSignatureR, ECC_STU_BIGINT32 *pSignatureS);

/*******************************************************************************
 * Function Name  : LIB_SM2_SignatureWithIDA
 * Description    : 使用私钥及身份信息对明文进行签名；
 *
 *                 备注：
 *                 1.此函数内部使用随机数模块，需要打开模块时钟;
 *                 2.此函数内部使用SHA模块，需要请打开模块时钟;
 *                 3.此函数内部使用EDMAC模块，需要请打开模块时钟;
 *                 4.此函数内部使用CRYPTO模块，需要请打开模块时钟;
 *
 * Input          : mes                     明文
 *                  klen                    明文字节长度
 *                  IDA                     签名方的身份信息
 *                  entla                   签名方身份信息字节长度
 *                  pstuEccPrams            椭圆曲线参数
 *                  pstuPubKey              公钥
 *                  pstuPrivKey             私钥
 *                  pSignatureR             输出明文的签名信息R
 *                  pSignatureS             输出明文的签名信息S
 * Output         : None
* Return         :None
 ******************************************************************************/
void LIB_SM2_SignatureWithIDA(unsigned char *mes, unsigned short klen, unsigned char *IDA, unsigned short entla, SM2_STU_PRAMS *pstuEccPrams, SM2_STU_PUBKEY *pstuPubKey, ECC_STU_BIGINT32 *pstuPrivKey, ECC_STU_BIGINT32 *pSignatureR, ECC_STU_BIGINT32 *pSignatureS);

/*******************************************************************************
 * Function Name  : LIB_SM2_VerificationWithIDA
 * Description    : 使用公钥及身份信息对明文和签名进行验证；
 *
 *                 备注：
 *                 1.此函数内部使用CRYPTO模块，需要打开模块时钟;
 *                 2.此函数内部使用SHA模块，需要打开模块时钟;
 *                 3.此函数内部使用EDMAC模块，需要打开模块时钟;
 *                 4.此函数内部使用CRYPTO模块，需要打开模块时钟;
 *
 * Input          : mes                   明文
 *                  klen                  明文字节长度
 *                  IDA                   签名方的身份信息
 *                  entla                 签名方身份信息的字节长度（备注：原来库为签名方身份信息的bit长度）
 *                  pstuEccPrams          椭圆曲线参数
 *                  pstuPubKey            公钥
 *                  pSignatureR           明文的签名信息R
 *                  pSignatureS           明文的签名信息S
 * Output         : None
 * Return         : 0 - 验证成功
 *                  1 - 验证失败
 ******************************************************************************/
UINT8 LIB_SM2_VerificationWithIDA(unsigned char *mes, unsigned short klen, unsigned char *IDA, unsigned short entla, SM2_STU_PRAMS *pstuEccPrams, SM2_STU_PUBKEY *pstuPubKey, ECC_STU_BIGINT32 *pSignatureR, ECC_STU_BIGINT32 *pSignatureS);

/*******************************************************************************
 * Function Name  : LIB_SM2_Verification
 * Description    : 使用公钥对明文和签名进行验证；
 *
 *                 备注：
 *                 1.此函数内部使用EDAMC模块，需要打开模块时钟;
 *                 2.此函数内部使用CRYPTO模块，需要打开模块时钟;
 *
 * Input          : mes                明文
 *                  klen               明文字节长度
 *                  pstuEccPrams       椭圆曲线参数
 *                  pstuPubKey         公钥
 *                  pSignatureR        明文的签名信息R
 *                  pSignatureS        明文的签名信息S
 * Output         : None
 * Return         : 0 - 验证成功
 *                  1 - 验证失败
 ******************************************************************************/
UINT8 LIB_SM2_Verification(unsigned char *mes, unsigned short klen, SM2_STU_PRAMS *pstuEccPrams, SM2_STU_PUBKEY *pstuPubKey, ECC_STU_BIGINT32 *pSignatureR, ECC_STU_BIGINT32 *pSignatureS);

/*******************************************************************************
 * Function Name  : LIB_AES_Enable
 * Description    : 使能AES模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_AES_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_AES_Disable
 * Description    : 禁用AES模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_AES_Disable(void);

/*******************************************************************************
* Function Name  : LIB_AES_Cryptographic
* Description    : 通过AES (128bits、192bits、256bits)进行加解密运算
* Input          : ende  - 加解密选择
*                      0 - 加密;
*                      1 - 解密.
*                  key_mode    - 算法密钥长度定义
*                  sel_mode    - 对称算法加解密模式选择
*                  stream_mode - 对称算法流模式选择
*                  data_in     - 加解密数据地址
*                  data_out    - 加解密结果地址
*                  dlen        - 加解密数据长度，单位为字节
*                  key         - 加解密密钥
*                  iv          - 初始向量值
*                  cnt         - CTR模式counter值
*                  bDPA        - 密钥防护等级
* Output         : None
* Return         : 0 - 运算成功
 *                 1 - 运算失败
******************************************************************************/
unsigned int LIB_AES_Cryptographic(unsigned char code, ALG_KEY_MODE key_mode, ALG_MODE sel_mode, ALG_STREAMMODE stream_mode, unsigned int *data_in,
                                   unsigned int *data_out, unsigned int dlen, unsigned int *key, unsigned int *iv, unsigned int *cnt, unsigned int bDPA);

/*******************************************************************************
 * Function Name  : LIB_DES_Enable
 * Description    : 使能DES模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_DES_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_DES_Disable
 * Description    : 禁用DES模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_DES_Disable(void);

/*******************************************************************************
* Function Name  : LIB_DES_Cryptographic
* Description    : 通过des/3des (64bits、128bits、192bits) 进行加解密运算
* Input          : ende - 加解密选择
*                      0 - 加密;
*                      1 - 解密.
*                  key_mode     - 算法密钥长度定义
*                  sel_mode     - 对称算法加解密模式选择
*                  stream_mode  - 对称算法流模式选择
*                  data_in      - 加解密数据地址
*                  data_out     - 加解密结果地址
*                  dlen         - 加解密数据长度，单位为字节
*                  key          - 加解密密钥
*                  iv           - 初始向量值，固定为8字节长度的值
*                  bDPA         - 密钥防护等级
* Output         : None
* Return         : 0 - 运算成功
 *                 1 - 运算失败
******************************************************************************/
unsigned int LIB_DES_Cryptographic(unsigned char code, ALG_KEY_MODE key_mode, ALG_MODE sel_mode, ALG_STREAMMODE stream_mode, unsigned int *data_in,
                                   unsigned int *data_out, unsigned int dlen, unsigned int *key, unsigned int *iv, unsigned char dpa);

/*******************************************************************************
 * Function Name  : LIB_SMS4_Enable
 * Description    : 使能SMS4模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SMS4_Enable(void);

/*******************************************************************************
 * Function Name  : LIB_SMS4_Disable
 * Description    : 禁用SMS4模块
 * Input          : None
 * Output         : None
 * Return         : None
 ******************************************************************************/
void LIB_SMS4_Disable(void);

/*******************************************************************************
* Function Name  : LIB_SM4_Cryptographic
* Description    : 通过SM4 (64bits、128bits、192bits) 进行加解密运算
* Input          : ende - 加解密选择
*                      0 - 加密;
*                      1 - 解密.
*                  sel_mode - 对称算法加解密模式选择
*                      0 - ECB模式;
*                      1 - CBC模式.
*                  stream_mode  - 对称算法流模式选择
*                  data_in      - 加解密数据地址
*                  data_out     - 加解密结果地址
*                  dlen         - 加解密数据长度，单位为字节
*                  key          - 加解密密钥
*                  iv           - 初始向量值，固定为8字节长度的值
*                  bDPA         - 密钥防护等级
* Return         : 0- 运算成功
 *                 1- 运算失败
******************************************************************************/
unsigned int LIB_SM4_Cryptographic(unsigned int code, ALG_MODE sel_mode, ALG_STREAMMODE strem_mode, unsigned int *data_in,
                                   unsigned int *data_out, unsigned int dlen, unsigned int *key, unsigned int *iv, dpa_level_t dpa);


/*******************************************************************************
* Function Name  : EccPointMul_New
* Description    : 点乘运算接口
                   pStuK*(pPointAx,pPointAy) =(pPointBx,pPointBy)
* Input          : pStuK  - 加解密选择
*                  pPointAx 输入点横坐标
*                  pPointAy 输入点纵坐标
*Output
*                  pPointBx 输出点横坐标
*                  pPointBy 输出点纵坐标

* Return         : none

******************************************************************************/
void EccPointMul_New(ECC_STU_BIGINT32 *pStuK,ECC_STU_BIGINT32 *pPointAx,ECC_STU_BIGINT32 *pPointAy,ECC_STU_BIGINT32 *pPointBx,ECC_STU_BIGINT32 *pPointBy,SM2_STU_PRAMS *pstuEccPrams);


/*******************************************************************************
* Function Name  : EccPointAdd
* Description    : 点加接口
                   (pPointAx,pPointAy) + (pPointBx,pPointBy) = (pPointCx,pPointCy)
* Input
*                  pPointAx 输入点横坐标
*                  pPointAy 输入点纵坐标
*                  pPointBx 输入点横坐标
*                  pPointBy 输入点纵坐标
*Output
*                  pPointCx 输出点横坐标
*                  pPointCy 输出点纵坐标

* Return         : none

******************************************************************************/
void EccPointAdd(ECC_STU_BIGINT32 *pPointAx,ECC_STU_BIGINT32 *pPointAy,ECC_STU_BIGINT32 *pPointBx,ECC_STU_BIGINT32 *pPointBy,ECC_STU_BIGINT32 *pPointCx,ECC_STU_BIGINT32 *pPointCy,SM2_STU_PRAMS *pstuEccPrams);

#endif /* __ALGO_H__ */
