"""Build a teacher-facing weekly report from a frozen measured review."""
import argparse
import json
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Table, TableStyle, PageBreak, Spacer


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('review',type=Path)
    ap.add_argument('output',type=Path)
    ap.add_argument('--comparison',type=Path)
    ap.add_argument('--followup',type=Path)
    args=ap.parse_args()
    data=json.loads(args.review.read_text())
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    cutoff=datetime.fromisoformat(data.get('audit_at', data['created_at'])).astimezone().strftime('%Y-%m-%d %H:%M %Z')
    cases=data['cases'];counts=data['category_counts'];labels=data['category_labels']
    comparison=json.loads(args.comparison.read_text()) if args.comparison else {}
    followup=json.loads(args.followup.read_text()) if args.followup else {}
    pdfmetrics.registerFont(TTFont('CN','/System/Library/Fonts/Supplemental/Arial Unicode.ttf'))
    styles={
        'body':ParagraphStyle('body',fontName='CN',fontSize=10.2,leading=16,spaceAfter=8,wordWrap='CJK',textColor=colors.HexColor('#253649')),
        'title':ParagraphStyle('title',fontName='CN',fontSize=22,leading=30,spaceAfter=14,wordWrap='CJK',textColor=colors.HexColor('#12334c')),
        'head':ParagraphStyle('head',fontName='CN',fontSize=13,leading=20,spaceBefore=10,spaceAfter=7,wordWrap='CJK',textColor=colors.HexColor('#15516c')),
        'small':ParagraphStyle('small',fontName='CN',fontSize=8.1,leading=11.5,spaceAfter=4,wordWrap='CJK',textColor=colors.HexColor('#253649')),
    }
    def p(text,style='body'):
        return Paragraph(escape(str(text)).replace('\n','<br/>'),styles[style])
    story=[]
    def add(text,style='body'):story.append(p(text,style))
    def table(headers,rows,widths):
        t=Table([[p(v,'small') for v in headers]]+[[p(v,'small') for v in r] for r in rows],colWidths=widths,repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8f0f4')),('VALIGN',(0,0),(-1,-1),'TOP'),('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor('#608297')),('LINEBELOW',(0,1),(-1,-1),.25,colors.HexColor('#dbe4e9')),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6)]))
        story.append(t)
    add('Crash2OpenX\n42 场景轨迹表现测试周报','title')
    add('2026 年 9 月 18 日（周五） | 数据截止：'+cutoff,'small')
    add('上周讨论的任务','head')
    add('从固定的42份原始事故报告PDF出发，通过现有Crash2OpenX框架生成对应的OpenDRIVE道路与OpenSCENARIO场景，在指定CARLA/PCLA服务器逐例测试车辆行为、交互和碰撞表现，整理每例场景文件、真实MP4及运行日志，并统计轨迹可用性及不足原因。')
    add('本周完成情况','head')
    add('已完成42例从原始PDF到新生成道路、场景及真实运行证据的对应整理，保留VLM读取、RoadSeed/SceneSeed推理、OCL/QA、道路编译、CARLA路网提取与XOSC编译记录。生成入口禁止读取旧compiled地图和seed，断点续跑仅使用本批已核对来源的生成结果；没有按编号编写轨迹作为生成输入。')
    add('本周重点测试停车避让、超车并回、路口转弯、多阶段纵向动作及连续接触等轨迹，并对原始越界指标进行独立道路几何复核。运行使用指定服务器上的CARLA与PCLA InterFuser；当前InterFuser在CPU上运行。')
    table(['统计口径','本版结果'],[
        ['有本批新XODR、XOSC、真实MP4和日志','42 / 42'],
        ['所选运行通过基本终止/高度检查（非运动质量验收）',f"{data['selected_execution_integrity_cases']} / 42"],
        ['所选地图有匹配哈希的原生CARLA几何一致性检查',f"{data.get('selected_native_geometry_verified_cases', '未统计')} / 42"],
        ['保存可核对的动作阶段观察',f"{data.get('selected_cases_with_stage_observations', '未统计')} / 42"],
        ['出现参与者横滚/俯仰超过60度的运行',str(data.get('selected_cases_with_orientation_excursions', '未统计'))+'；另行核对运动质量'],
        ['完整原文语义与轨迹共同验收','0 / 42；尚未建立完全可用结论'],
        ['自主驾驶任务成功率','尚未建立；路线进度/无碰撞不能代替任务成功'],
    ],[350,150])
    add('42套证据齐备不等于42例成功。严格事故重建口径下，本版没有可宣称“完全可用”的场景；部分行驶、停车或接触动作已有可用观测，具体不足逐例列出。','small')
    story.append(PageBreak())
    add('不同轨迹的表现与原因','title')
    add('对每例所选运行按以下顺序归类，一例只计一类：运行完整性、中心越界/逆行、接触、车道边界接触、路线进度。≥90%仅作为描述高进度观测的阈值，不是通过标准。','small')
    table(['轨迹表现分类','数量','编号'],[[labels[k],str(n),'、'.join(r['case_id'].split('_')[0] for r in cases if r['category']==k)] for k,n in counts.items()],[255,35,210])
    add('代表性结果','head')
    add(followup.get('case113_text', '113历史已测版本：观察到首次接触后减速、真实分离、自车停稳后再次加速并接触。阶段复核选出约8.07秒与41.07秒的接触，仍需核对持续接触和撞击部位。该历史版本传感器562条记录不能写成562次事故；当前选用版本见逐例清单。'))
    add('274历史对照：生成停车带并保留“加速、进入停车带、并回”的多阶段动作。无标线运行与低亮度标线运行均未出现预期接触；后者车道边界接触由7次降至0次，路线进度由65.5%升至94.5%。这是单次对照，尚不能归因为标线；本版最新所选运行另见逐例清单。')
    if comparison:
        results=comparison.get('runs',{})
        table(['330历史同地图/SceneSeed对照','无标线','带标线'],[
            ['中心越界原始时长（秒）',results.get('control',{}).get('off_road_time','待核对'),results.get('marked',{}).get('off_road_time','待核对')],
            ['路线进度（%）',results.get('control',{}).get('final_route_completion','待核对'),results.get('marked',{}).get('final_route_completion','待核对')],
            ['碰撞传感器记录',results.get('control',{}).get('collision_count','待核对'),results.get('marked',{}).get('collision_count','待核对')],
        ],[280,110,110])
        add('330表中数值来自历史控制变量对照；本版所选运行使用后续新生成地图，其数值见逐例清单，不能与此表混作标线效果比较。','small')
    add(followup.get('case259_text', '259：保留两种失败证据。一版运行32帧后场景失败；较新版本运行1270帧但行人跌出生成地图，不能选作合格轨迹。')+followup.get('case038_text','038的部分侵道与骑行者关系仍待共同验收，较早实录不能代表完整原文语义。'),'small')
    story.append(PageBreak())
    add('评价边界与下一步','title')
    add('区分四类结果','head')
    table(['层次','本版结论'],[
        ['生成','42例累计有经来源检查的新道路/场景；版本不完全相同，逐例记录。'],
        ['真实运行','42例有CARLA实录和日志；失败与中止同样是证据，不计作轨迹成功。'],
        ['事故重建','需要参与者、先后顺序、接触对象/部位及运动过程匹配原文；当前未建立完整通过案例。'],
        ['自主驾驶','PCLA保持自主控制，所选SUT未必是报告中的AV；避让无碰撞与事故重建分别评价。'],
    ],[75,425])
    add('为什么部分轨迹不理想','head')
    add('停车/停滞类常在接近目标后停止，未完成源报告中的追尾、超车或侧擦；部分路口和转向轨迹有实际越界或逆向记录；发生接触的场景还可能碰到错误参与者（如013汽车与自行车），或缺少有序的多次碰撞。原报告通常没有精确速度、间距等信息，初始化默认值不应被当作已知事实。')
    conflict=[r['case_id'].split('_')[0] for r in cases if r['raw_projection_conflict']]
    if conflict:
        add('独立解析XODR复核发现，'+'、'.join(conflict)+'的原始越界时间与全部已记录自车中心在设计驾驶车道内的结果不一致。两类指标并列保留；中心在车道内不证明整车轮廓在内，也不证明车道选择合法。')
    else:
        add('本版所选运行未出现“原始指标报越界、独立解析XODR复核全部自车中心在设计驾驶车道内”的冲突。两类指标继续并列保留；中心在车道内不证明整车轮廓在内，也不证明车道选择合法。')
    add('CSV新增逐例原生CARLA几何检查状态；只有与该例地图哈希相符且通过坐标采样检查，才计入几何一致。其余版本的设计道路投影仅作诊断。基本运行完整性检查允许停滞终止，且不等于“日志无告警”；结束后的传感器清理状态另列。','small')
    add('CSV同时列出动作阶段和姿态异常。START/END只证明行为树阶段，不证明物理动作成功；未采集阶段记录也不能证明动作未发生。013汽车约79.40秒处于横滚或俯仰超过60度的姿态，虽未跌出地图，运动过程仍明显异常。','small')
    add('生成道路网格缺少真实街景、部分交通设施及清晰标线；低亮度标线使用CARLA世界调试线进入真实相机画面，属于明确的渲染近似，不是原生道路材质。当前对照是单次运行，不支持因果或统计显著性结论。')
    if followup:
        add('原生CARLA检查用于核对实际导入道路与设计XODR。本版未完成此项验证的地图仍仅有设计道路诊断，不能据此证明车辆在实际CARLA路面内；历史几何不一致及失败版本的证据保留在补充材料中。','small')
    add('后续工作','head')
    add('继续按原报告逐项测试不同轨迹的表现，优先推进部分侵道、行人横穿后的终止位置、碰撞后旋转、有序多次接触和自主避让停滞场景。对照试验保持生成地图与SceneSeed一致，并增加重复运行。完成源语义、实际轨迹及碰撞证据共同验收后再更新“完全可用”数量。')
    add('后续将制作一份PPT，系统讲解这篇已中稿论文的研究问题、方法流程、实验设计，以及本次42场景测试的结果和限制。')
    add('交付使用说明','head')
    add('压缩包按原编号组织，每例含map.xodr、scenario.xosc、原编译XOSC、真实运行XOSC、carla_rgb.mp4、日志、轨迹和来源证据。scenario.xosc仅调整地图引用便于解压配对，记录重定位哈希；运行仍需CARLA/PCLA环境。CSV列出对应路径与每例原因。原始日志未覆盖。','small')
    if followup:
        story.append(PageBreak())
        add('路口几何与轨迹补充测试','title')
        for section in followup['sections']:
            add(section['title'],'head')
            add(section['text'])
        add('以上为本周新增观测，不将单次测试或地图检查计作完整事故重建通过。','small')
    for page in range(3):
        story.append(PageBreak());add(f'逐例测试清单 {page+1}/3','title')
        add('全部案例的完整事故验收均未建立；下表描述已测表现。边界=车道侵入传感器记录；越界=解析道路复核的自车中心越界帧数。','small')
        rows=[]
        for r in cases[page*14:(page+1)*14]:
            measured=f"{r['total_ticks']}帧 / 进度{r['route_progress_percent']:.1f}%\n接触{r['contact_sensor_records']}；边界{r['lane_boundary_contacts']}；越界{r['center_outside_samples']}帧"
            rows.append([r['case_id'].split('_')[0],r['source_obligations'],measured,r['category_zh']])
        table(['编号','原文关键动作','真实运行观测','轨迹表现'],rows,[32,219,134,115])
    def footer(canvas,doc):
        canvas.setFont('CN',8);canvas.setFillColor(colors.HexColor('#617786'))
        canvas.drawString(47,26,'Crash2OpenX | 轨迹表现测试 | 阶段性报告')
        canvas.drawRightString(A4[0]-47,26,str(doc.page))
    path=out/'Crash2OpenX_trajectory_weekly_20260918.pdf'
    SimpleDocTemplate(str(path),pagesize=A4,rightMargin=47,leftMargin=47,topMargin=40,bottomMargin=44,title='Crash2OpenX 42场景轨迹表现测试周报',author='Xijun Liu').build(story,onFirstPage=footer,onLaterPages=footer)
    email=f'''老师您好：

向您汇报本周Crash2OpenX的42场景轨迹表现测试进度。

上周讨论的任务是将固定42份事故报告PDF通过现有框架生成OpenDRIVE/OpenSCENARIO，在指定CARLA/PCLA服务器逐例测试，并整理对应场景文件、真实MP4与运行日志。

截至{cutoff}，42例的新生成文件与真实运行证据已齐备，逐阶段来源及版本已记录。本周主要测试停车避让、超车并回、转弯、多阶段动作与连续接触等轨迹。按本版所选运行，{data['selected_execution_integrity_cases']}例通过基本终止/高度检查，此项不代表运动质量验收；完整事故语义与轨迹共同验收目前为0/42，不能把证据齐备写成全部场景完全可用。

已按轨迹表现逐例分类并说明原因：部分场景能稳定观察行驶和停车，但未复现原事故；另有越界、停滞、接触对象不一致、多次碰撞不完整等情况。274例已观察到进入停车带后并回，历史标线对照中侵线次数由7次降为0次，但未发生预期碰撞。113历史版本观察到分离、停稳、再加速与再次接触，持续接触及部位仍待验收。259例存在短时失败及行人跌出地图的不同版本，均如实保留。报告同时区分生成、运行、事故重建和自主驾驶结果，路线进度或无碰撞不代替任务成功率。

报告及42例CSV列明逐项状态，配套压缩包按编号对应场景文件、录像、日志和生成证据。下一步继续测试不同轨迹的表现，推进源语义与实际轨迹共同验收，并增加控制变量对照。

后续我会再制作一份PPT，系统讲解这篇已中稿论文的方法、整体流程、实验设计及当前验证结果。

谢谢老师！
Xijun
'''
    if followup:
        email=email.replace('113历史版本观察到分离、停稳、再加速与再次接触，持续接触及部位仍待验收。', followup.get('case113_email_text', '113历史版本观察到分离、停稳、再加速与再次接触，持续接触及部位仍待验收。'))
        email=email.replace('259例存在短时失败及行人跌出地图的不同版本，均如实保留。', followup['case259_text'])
        email=email.replace('报告及42例CSV列明逐项状态，', followup['email_summary']+'\n\n报告及42例CSV列明逐项状态，')
        (out/'followup_snapshot.json').write_text(json.dumps(followup,ensure_ascii=False,indent=2)+'\n')
    (out/'email_body.txt').write_text(email)
    (out/'email_status.json').write_text(json.dumps({'state':'not_sent','reason':'recipient and usable sending account not yet provided','subject':'Crash2OpenX：42场景轨迹表现测试周报（2026-09-18）'},ensure_ascii=False,indent=2)+'\n')
    (out/'report_snapshot.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    (out/'trajectory_status_42.csv').write_bytes(args.review.with_name('trajectory_status_42.csv').read_bytes())
    print(path)


if __name__=='__main__':
    main()
