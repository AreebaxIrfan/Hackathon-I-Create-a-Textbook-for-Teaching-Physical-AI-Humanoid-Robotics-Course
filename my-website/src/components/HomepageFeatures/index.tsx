import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'From Digital Brain to Physical Body',
    description: (
      <>
        Master the transition from pure digital AI to embodied intelligence that understands physics, gravity, and real-world interaction.
      </>
    ),
  },
  {
    title: 'Hands-On with Real & Simulated Humanoids',
    description: (
      <>
        Build, simulate, and deploy autonomous humanoids using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action models — all in one curriculum.
      </>
    ),
  },
  {
    title: 'Complete Lab-Ready Guide',
    description: (
      <>
        Full hardware BOMs (RTX workstation + Jetson kits + Unitree robots), cloud alternatives, and economy $700 student kits included.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4 ')}>
      {/* <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div> */}
      <div className="text--center padding-horiz--md ">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
